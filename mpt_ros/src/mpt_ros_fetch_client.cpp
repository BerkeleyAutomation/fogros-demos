#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <Eigen/Dense>
#include <sstream>
#include <fstream>

std::vector<unsigned char> loadFile(const std::string& fileName) {
    std::string resourceDir = "/home/ubuntu/catkin_ws/src/mpt_ros/resources/mplambda/";
    std::vector<unsigned char> data;
    std::ifstream file(resourceDir + fileName, std::ios::binary);
    file.unsetf(std::ios::skipws); // don't skip newlines
    file.seekg(0, std::ios::end); // seek to end to get file size
    data.reserve(file.tellg()); // reserve bytes for file
    file.seekg(0, std::ios::beg); // seek to beginning to start read.
    data.insert(data.begin(), std::istream_iterator<unsigned char>(file),
                std::istream_iterator<unsigned char>());
    return data;
}

void sendMesh(ros::Publisher& pub, const std::string& file) {
    std_msgs::UInt8MultiArray msg;
    msg.data = loadFile(file);
    msg.layout.data_offset = 0;
    msg.layout.dim.emplace_back();
    msg.layout.dim[0].label = "dae";
    msg.layout.dim[0].size = msg.data.size();
    msg.layout.dim[0].stride = 1;
    pub.publish(msg);
}

template <class Derived>
void sendMatrix(ros::Publisher& pub, const Eigen::MatrixBase<Derived>& m) {
    std_msgs::Float64MultiArray msg;
    msg.data.reserve(m.rows() * m.size());
    for (int c=0 ; c<m.cols() ; ++c)
        for (int r=0 ; r<m.rows() ; ++r)
            msg.data.push_back(m(r, c));
    msg.layout.data_offset = 0;
    msg.layout.dim.emplace_back();
    msg.layout.dim[0].size = m.rows();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim.emplace_back();
    msg.layout.dim[1].size = m.cols();
    msg.layout.dim[1].stride = 1;
    pub.publish(msg);
}

static void motionPlanCallback(const std_msgs::Float64MultiArray& msg) {
    ROS_INFO("Got Motion Plan");
    if (msg.layout.dim.size() != 2) {
        ROS_INFO("Motion planning failed");
    } else {
        ROS_INFO("Motion plan has %u waypoints, %zu bytes",
                 msg.layout.dim[0].size,
                 msg.data.size() * sizeof(msg.data[0]));

        int rows = msg.layout.dim[0].size;
        int cols = msg.layout.dim[1].size;
        
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m(rows, cols);
        
        for (unsigned i=0 ; i<rows ; ++i)
            for (unsigned j=0 ; j<cols ; ++j)
                m(i,j) = msg.data[i * msg.layout.dim[0].stride + j];

        std::ostringstream str;
        str << m;
        ROS_INFO("Waypoints: \n%s", str.str().c_str());
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "mpt_client");

    ros::NodeHandle n;

    // std::string envFile;
    // std::string robotFile;
    
    // n.getParam("env", envFile);
    // n.getParam("robot", robotFile);

    
    ros::Publisher envMeshPub = n.advertise<std_msgs::UInt8MultiArray>("fetch_world_mesh", 1000);
    ros::Publisher planRequestPub = n.advertise<std_msgs::UInt8MultiArray>("fetch_plan_request", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        if (envMeshPub.getNumSubscribers() > 0 &&
            planRequestPub.getNumSubscribers() > 0)
            break;
        loop_rate.sleep();
    }

    double PI = 3.1415926536;
    double PI_2 = 1.5707963268;
    
    if (ros::ok()) {
        ROS_INFO("Sending env");
        Eigen::Matrix<double, 6, 1> envFrame;
        Eigen::Matrix<double, 8, 1> startConfig;
        Eigen::Matrix<double, 6, 1> goal;
        Eigen::Matrix<double, 6, 1> goalRadius;

        int taskNo = 1;
        if (taskNo == 1) {
// ./mpl_robot \
//     --scenario fetch \
//     --algorithm cforest \
//     --coordinator "$COORDINATOR" \
//     --jobs 8 \
//     --env AUTOLAB.dae \
//     --env-frame=0.57,-0.90,0.00,0,0,-$PI_2 \
//     --goal=-1.07,0.16,0.88,0,0,0 \
//     --goal-radius=0.01,0.01,0.01,0.01,0.01,$PI \
//     --start=0.1,$PI_2,$PI_2,0,$PI_2,0,$PI_2,0 \
//     --time-limit 300 \
//     --check-resolution 0.01
            sendMesh(envMeshPub, "AUTOLAB.dae");
            envFrame << 0.57,-0.90,0.00,0,0,PI_2;
            startConfig << 0.1, PI_2, PI_2, 0, PI_2, 0, PI_2, 0;
            goal << -1.07, 0.16, 0.88, 0, 0, 0;
            goalRadius << 0.01, 0.01, 0.01, 0.01, 0.01, PI;
            
            
        } else if (taskNo == 2) {
            
// START=0.18132,0.249076,0.153913,1.47009,1.48902,-0.125871,-1.74715,-1.45724
// # GOAL=0.38615,-1.08792,0.0778814,-1.02092,-0.820869,3.05061,-0.295709,-1.93488
// GOAL=0.70,-0.65,1.3,0,0,0
// GOALR=0.01,0.01,0.01,0.01,$PI,$PI
// # --goal-radius=0.01,0.01,0.01,0.01,0.01,$PI \
    
// ./mpl_robot \
//     --scenario fetch \
//     --algorithm rrt \
//     --coordinator "$COORDINATOR" \
//     --jobs 1 \
//     --env resources/AUTOLAB.dae \
//     --env-frame=0.48,1.09,0.00,0,0,-$PI_2 \
//     --goal=$GOAL \
//     --goal-radius=$GOALR \
//     --start=$START \
//     --time-limit 120 \
//     --check-resolution 0.01
            sendMesh(envMeshPub, "AUTOLAB.dae");
            envFrame << 0.48,1.09,0.00,0,0, PI_2;
            startConfig << 0.18132,0.249076,0.153913,1.47009,1.48902,-0.125871,-1.74715,-1.45724;
            goal << 0.70,-0.65,1.3,0,0,0;
            goalRadius << 0.01,0.01,0.01,0.01,PI,PI;
        } else {
            throw std::invalid_argument("unknown scenario: " + std::to_string(taskNo));
        }

        Eigen::Matrix<double, 6+8+6+6, 1> planRequest;
        planRequest << envFrame, startConfig, goal, goalRadius;

        sendMatrix(planRequestPub, planRequest);
    
        // int count = 0;
        // while (ros::ok()) {
        //     std_msgs::String msg;

        //     std::stringstream str;
        //     str << "Hello! " << count;
        //     msg.data = str.str();

        //     ROS_INFO("%s", msg.data.c_str());

        //     chatter_pub.publish(msg);

        //     ros::spinOnce();
            
        //     loop_rate.sleep();
        //     ++count;
        // }
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    
    return 0;
}
