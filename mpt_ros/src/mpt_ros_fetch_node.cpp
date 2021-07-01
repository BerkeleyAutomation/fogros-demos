#include <cassert>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fcl/fcl.h>
#include <mpt/discrete_motion_validator.hpp>
#include <mpt/goal_state.hpp>
#include <mpt/se3_space.hpp>
#include <mpt/uniform_sampler.hpp>
#include <mpt/prrt_star.hpp>
#include <nigh/kdtree_batch.hpp>
#include <mpt_ros/mpl/demo/fetch_scenario.hpp>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"

using Clock = std::chrono::steady_clock;

namespace mpt_ros {
    using namespace unc::robotics;

    template <class S>
    using Mesh = fcl::BVHModel<fcl::OBBRSS<S>>;

    template <class S>
    auto mapToEigen(const aiMatrix4x4t<S>& m) {
        using Matrix = const Eigen::Matrix<S, 4, 4, Eigen::RowMajor>;
        static_assert(sizeof(Matrix) == sizeof(m));
        return Eigen::Map<Matrix>(&m.a1);
    }

    template <class S>
    auto mapToEigen(const aiVector3t<S>& v) {
        using Vector = const Eigen::Matrix<S, 3, 1>;
        static_assert(sizeof(Vector) == sizeof(v));
        return Eigen::Map<Vector>(&v.x);
    }

    template <class S, int mode>
    std::pair<Eigen::Matrix<S, 3, 1>, std::size_t>
    computeMeshCenter(
        const aiScene *scene, const aiNode *node,
        Eigen::Transform<S, 3, mode> transform)
    {
        Eigen::Matrix<S, 3, 1> center = Eigen::Matrix<S, 3, 1>::Zero();
        std::size_t count = 0;
        transform *= mapToEigen(node->mTransformation).template cast<S>();
        for (unsigned i=0 ; i<node->mNumMeshes ; ++i) {
            const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            count += mesh->mNumVertices;
            for (unsigned j=0 ; j<mesh->mNumVertices ; ++j)
                center += transform * mapToEigen(mesh->mVertices[j]).template cast<S>();
        }
        for (unsigned i=0 ; i<node->mNumChildren ; ++i) {
            auto [childCenter, childCount ] = computeMeshCenter(scene, node->mChildren[i], transform);
            center += childCenter;
            count += childCount;
        }
        return { center, count };
    }

    template <class S>
    void addNodeToMesh(const aiScene *scene, const aiNode *node, fcl::Transform3<S> transform, Mesh<S>& model) {
        using Vec3 = Eigen::Matrix<S, 3, 1>;
        transform *= mapToEigen(node->mTransformation).template cast<S>();
        MPT_LOG(INFO) << "  transform = " << transform.matrix();
        for (unsigned i=0 ; i<node->mNumMeshes ; ++i) {
            const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            for (unsigned j=0 ; j<mesh->mNumFaces ; ++j) {
                const aiFace& face = mesh->mFaces[j];
                if (face.mNumIndices < 3)
                    continue;

                // Support trangular decomposition by fanning out
                // around vertex 0.  The indexing follows as:
                //
                //   0---1   0 1 2
                //  /|\ /    0 2 3
                // 4-3-2     0 3 4
                //
                Vec3 v0 = transform * mapToEigen(mesh->mVertices[face.mIndices[0]]).template cast<S>();
                Vec3 v1 = transform * mapToEigen(mesh->mVertices[face.mIndices[1]]).template cast<S>();
                for (unsigned k=2 ; k<face.mNumIndices ; ++k) {
                    Vec3 v2 = transform * mapToEigen(mesh->mVertices[face.mIndices[k]]).template cast<S>();
                    model.addTriangle(v0, v1, v2);
                    v1 = v2;
                }
            }
        }
        for (unsigned i=0 ; i<node->mNumChildren ; ++i)
            addNodeToMesh(scene, node->mChildren[i], transform, model);
    }
    
    template <class S>
    std::shared_ptr<Mesh<S>> loadMesh(
        const void *pBuffer, std::size_t pLength, const char *pHint, bool shiftToCenter)
    {
        using Transform = fcl::Transform3<S>;
        Assimp::Importer importer;

        static constexpr auto readOpts
            = aiProcess_Triangulate
            | aiProcess_JoinIdenticalVertices
            | aiProcess_SortByPType
            | aiProcess_OptimizeGraph
            | aiProcess_OptimizeMeshes
            ;

        const aiScene *scene = importer.ReadFileFromMemory(
            pBuffer, pLength, readOpts, pHint);

        if (scene == nullptr)
            throw std::invalid_argument("could not load mesh");

        if (!scene->HasMeshes())
            throw std::invalid_argument("no meshes in data");

        Transform rootTransform = Transform::Identity();

        if (shiftToCenter) {
            auto [ sum, count ] = computeMeshCenter(scene, scene->mRootNode, Transform::Identity());
            MPT_LOG(INFO) << "Shift to center " << sum << " / " << count << " = " << (sum/count);
            rootTransform *= Eigen::Translation<S, 3>(sum / -count);
        }

        std::shared_ptr<Mesh<S>> mesh = std::make_shared<Mesh<S>>();
        mesh->beginModel();
        addNodeToMesh(scene, scene->mRootNode, rootTransform, *mesh);
        mesh->endModel();
        mesh->computeLocalAABB();
        return mesh;
    }

    template <class S>
    class MPTFetchNode {
        using Scenario = mpl::demo::FetchScenario<S>;
        using State = typename Scenario::State;
        using Frame = typename Scenario::Frame;
        using Mesh = mpt_ros::Mesh<S>;
        static constexpr int DIMENSIONS = 8;
        
        // std::shared_ptr<Mesh> environment_;
        // std::shared_ptr<Mesh> robot_;

        // bool haveBounds_{false};
        // Eigen::Matrix<S, Eigen::Dynamic, 1> vMin_;
        // Eigen::Matrix<S, Eigen::Dynamic, 1> vMax_;

        // bool haveStartAndGoal_{false};
        // State qStart_;
        // State qGoal_;

        ros::NodeHandle nh_;
        ros::Subscriber envMeshSub_;
        ros::Subscriber envFrameSub_;
        ros::Subscriber planStartSub_;
        ros::Subscriber planGoalSub_;
        ros::Publisher motionPlanPub_;

        struct MotionPlanRequest {
            Frame envFrame_;
            std::shared_ptr<Mesh> environment_;
            State qStart_{State::Zero()};
            Frame goalFrame_{Frame::Identity()};
            Eigen::Matrix<S,6,1> goalRadius_{Eigen::Matrix<S,6,1>::Constant(0.1)};
            S checkResolution_{0.1};

            MotionPlanRequest() {
            }
            
            MotionPlanRequest(
                const Frame& envFrame,
                const std::shared_ptr<Mesh>& env,
                const State& qStart,
                const Frame& goalFrame)
                : envFrame_(envFrame)
                , environment_(env)
                , qStart_(qStart)
                , goalFrame_(goalFrame)
            {
            }
        };

        MotionPlanRequest motionPlanRequest_;
        bool hasPlanRequest_{false};

        // We implemente a 1-item queue between callbacks and planning
        // thread based on a std::optional.  We also overwrite this
        // queue entry with the latest request so that when the
        // planning thread is ready, it will run the most recent plan.
        std::mutex runStateMutex_;
        std::atomic<bool> shutdown_{false};
        std::optional<MotionPlanRequest> nextRequest_;
        std::condition_variable nextRequestReady_;

        std::thread planThread_;

        template <class Algorithm>
        void runPlanner(const Scenario& scenario, const typename Scenario::State& qStart) {
            mpt::Planner<Scenario, Algorithm> planner(scenario);
            planner.addStart(qStart);
            
            auto startTime = Clock::now();
            planner.solve([&] () { return shutdown_ || planner.solved(); });
            auto elapsed = Clock::now() - startTime;

            ROS_INFO("Planner ran for %lf seconds", std::chrono::duration<double>(elapsed).count());

            std_msgs::Float64MultiArray msg;
            msg.layout.data_offset = 0;
            msg.layout.dim.emplace_back();
            if (!planner.solved()) {
                msg.layout.dim[0].label = "failed";
                msg.layout.dim[0].size = 0;
                msg.layout.dim[0].stride = 7;
            } else {
                std::vector<State> path = planner.solution();
                msg.layout.dim.emplace_back();
                msg.layout.dim[0].label = "plan";
                msg.layout.dim[0].size = path.size();
                msg.layout.dim[0].stride = 7;
                msg.layout.dim[1].label = "waypoint";
                msg.layout.dim[1].size = 7;
                msg.data.reserve(path.size() * DIMENSIONS);
                for (std::size_t i=0 ; i<path.size() ; ++i)
                    for (int j=0 ; j<DIMENSIONS ; ++j)
                        msg.data.push_back(path[i][j]);
            }

            motionPlanPub_.publish(msg);
        }

        void plannerLoop() {
            // using Algorithm = mpt::PRRTStar<nigh::KDTreeBatch<>, mpt::single_threaded>;
            using Algorithm = mpt::PRRTStar<nigh::KDTreeBatch<>, mpt::hardware_concurrency>;

            ROS_INFO("Planner loop starting");
                            
            std::unique_lock<std::mutex> lock(runStateMutex_);
            while (!shutdown_) {
                nextRequestReady_.wait(lock);
                ROS_INFO("Planner loop awake");
                if (nextRequest_) {
                    S checkResolution = 1e-3;
                    Scenario scenario(
                        nextRequest_->envFrame_,
                        std::move(nextRequest_->environment_),
                        nextRequest_->goalFrame_,
                        nextRequest_->goalRadius_,
                        nextRequest_->checkResolution_);
                    State qStart = nextRequest_->qStart_;
                    nextRequest_.reset();

                    lock.unlock();
                    runPlanner<Algorithm>(scenario, qStart);
                    lock.lock();
                }
            }
        }
    
        auto meshCallback(const std::string& type, const std_msgs::UInt8MultiArray& msg, bool shiftToCenter) {
            ROS_INFO("Got %s, %d, %d, %s", type.c_str(), (int)msg.data.size(), (int)msg.layout.dim.size(),
                     msg.layout.dim.size() ? msg.layout.dim[0].label.c_str() : "(none)");
            //msg.layout.size()
            auto startLoad = Clock::now();
            auto mesh = mpt_ros::loadMesh<double>(
                msg.data.data(), msg.data.size(), msg.layout.dim[0].label.c_str(), shiftToCenter);
            auto elapsed = Clock::now() - startLoad;
            ROS_INFO("Loaded mesh in %lf seconds, tris=%d, verts=%d",
                     std::chrono::duration<double>(elapsed).count(),
                     mesh->num_tris, mesh->num_vertices);
            return mesh;
        }

        static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> loadMatrix(
            const std::string& name, const std_msgs::Float64MultiArray& msg) 
        {
            int rows = 0;
            int cols = 0;
                
            if (msg.layout.dim.size() == 2) {
                rows = msg.layout.dim[0].size;
                cols = msg.layout.dim[1].size;
            }
            
            ROS_INFO("Got %s, %dx%d=%d", name.c_str(), rows, cols, (int)msg.data.size());
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m(rows, cols);
            for (int c=0, i=0 ; c<cols ; ++c)
                for (int r=0 ; r<rows ; ++r, ++i)
                    m(r, c) = msg.data[i];
            
            return m;
        }
        
        template <class Source>
        static void convertToState(
            const Eigen::MatrixBase<Source>& src,
            State& dst)
        {
            dst = src;
            MPT_LOG(INFO) << "Got state: " << dst;
        }

        static Frame convertToFrame(const geometry_msgs::Pose& pose) {
            Frame f = Frame::Identity();
            f.translation()[0] = pose.position.x;
            f.translation()[1] = pose.position.y;
            f.translation()[2] = pose.position.z;
            f.linear() = Eigen::Quaternion<S>(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z).toRotationMatrix();
            return f;
        }

        template <class Derived>
        static Frame convertToFrame(const Eigen::MatrixBase<Derived>& q) {
            Eigen::Matrix<S, 3, 1> axis;
            Frame t = Frame::Identity();
            axis << q[3], q[4], q[5];
            S angle = axis.norm();
            if (std::abs(angle) > 1e-6) {
                Eigen::AngleAxis<S> aa(angle, axis / angle);
                t.linear() = aa.toRotationMatrix();
            }
            t.translation() << q[0], q[1], q[2];
            return t;
        }

        void optRunPlanner() {
            if (motionPlanRequest_.environment_ && hasPlanRequest_)
            {
                hasPlanRequest_ = false;

                std::unique_lock<std::mutex> lock(runStateMutex_);
                bool notify = !nextRequest_;
                nextRequest_.emplace(motionPlanRequest_);
                lock.unlock();

                // only notify if there wasn't a plan already.
                if (notify)
                    nextRequestReady_.notify_one();
            }
        }
        
        void environmentMeshCallback(const std_msgs::UInt8MultiArray& msg) {
            motionPlanRequest_.environment_ = meshCallback("environment", msg, false);
            optRunPlanner();
        }

        void planRequestCallback(const std_msgs::Float64MultiArray& msg) {
            auto parsed = loadMatrix("plan request", msg);
            if (parsed.rows() == 6+8+6+6 && parsed.cols() == 1) {
                motionPlanRequest_.envFrame_ = convertToFrame(parsed.col(0).head<6>());
                motionPlanRequest_.qStart_ = parsed.col(0).segment<8>(6); // start config
                motionPlanRequest_.goalFrame_ = convertToFrame(parsed.col(0).segment<6>(6+8)); // goal frame
                motionPlanRequest_.goalRadius_ = parsed.col(0).segment<6>(6+8+6); // goal radius
                hasPlanRequest_ = true;
                optRunPlanner();
            }
        }
        
        // void environmentFrameCallback(const geometry_msgs::Pose& msg) {
        //     motionPlanRequest_.envFrame_ = convertToFrame(msg);
        //     planHasEnvFrame_ = true;
        //     optRunPlanner();
        // }

        // void planStartCallback(const std_msgs::Float64MultiArray& msg) {
        //     auto start = loadMatrix("start", msg);
        //     if (start.rows() == DIMENSIONS && start.cols() == 1) {
        //         motionPlanRequest_.qStart_ = start;
        //         planHasStart_ = true;
        //         MPT_LOG(INFO) << "GOT start " << motionPlanRequest_.qStart_;
        //         optRunPlanner();
        //     }
        // }

        // void planGoalCallback(const geometry_msgs::Pose& msg) {
        //     motionPlanRequest_.goalFrame_ = convertToFrame(msg);
        //     MPT_LOG(INFO) << "GOT goal " << motionPlanRequest_.goalFrame_.matrix();
        //     planHasGoal_ = true;
        //     optRunPlanner();
        // }

    public:
        
        MPTFetchNode()
            : envMeshSub_(nh_.subscribe("fetch_world_mesh", 1000, &MPTFetchNode::environmentMeshCallback, this))
            , envFrameSub_(nh_.subscribe("fetch_plan_request", 1000, &MPTFetchNode::planRequestCallback, this))
            , motionPlanPub_(nh_.advertise<std_msgs::Float64MultiArray>("fetch_motion_plan", 1000))
            , planThread_(&MPTFetchNode::plannerLoop, this)
        {
        }

        ~MPTFetchNode() {
            // tODO: shutdown sequence (Set shutdown = true, wait for thread)
            shutdown_ = true;
            nextRequestReady_.notify_one();
            //std::unique_lock<std::mutex> lock(nextRequestReady_);
            planThread_.join();
        }
            
    };
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "mpt_ros");
    mpt_ros::MPTFetchNode<double> node;
    // ros::NodeHandle n;

    // // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    // ros::Subscriber envMeshSub = n.subscribe("environment_mesh", 1000, environmentMeshCallback);
    // ros::Subscriber robotMeshSub = n.subscribe("robot_mesh", 1000, robotMeshCallback);
    // ros::Subscriber boundsSub = n.subscribe("environment_bounds", 1000, environmentBoundsCallback);
    // ros::Subscriber planSub = n.subscribe("plan_start_to_goal", 1000, planStartToGoalCallback);

    ROS_INFO("MPT node started");
    
    ros::spin();
    
    return 0;
}
