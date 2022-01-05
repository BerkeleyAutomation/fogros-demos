#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Copyright ©2017. The Regents of the University of California (Regents).
All Rights Reserved. Permission to use, copy, modify, and distribute this
software and its documentation for educational, research, and not-for-profit
purposes, without fee and without a signed licensing agreement, is hereby
granted, provided that the above copyright notice, this paragraph and the
following two paragraphs appear in all copies, modifications, and
distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
otl@berkeley.edu,
http://ipira.berkeley.edu/industry-info for commercial licensing opportunities.

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL,
INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF
THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS BEEN
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

ROS Server for planning GQ-CNN grasps.

Author
-----
Mike Danielczuk, Vishal Satish & Jeff Mahler
"""
import json
import os
from timeit import default_timer as timer

import numpy as np
import rospy

from autolab_core import YamlConfig, CameraIntrinsics, ColorImage, DepthImage, BinaryImage, RgbdImage
from gqcnn.grasping import (RgbdImageState,
                            FullyConvolutionalGraspingPolicyParallelJaw,
                            FullyConvolutionalGraspingPolicySuction,
                            SuctionPoint2D, Grasp2D)
from gqcnn.utils import GripperMode, NoValidGraspsException

import message_filters
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from gqcnn_ros.msg import GQCNNGrasp


class GraspPlanner(object):
    def __init__(self, cfg, grasping_policy):
        """
        Parameters
        ----------
        cfg : dict
            Dictionary of configuration parameters.
        grasping_policy: :obj:`GraspingPolicy`
            Grasping policy to use.
        """
        self.cfg = cfg
        self.grasping_policy = grasping_policy

        # Create publishers to publish pose and q value of final grasp.
        self.grasp_publisher = rospy.Publisher("~grasp",
                                               GQCNNGrasp,
                                               queue_size=10)
        self.grasp_pose_publisher = rospy.Publisher("~grasp_pose",
                                                    PoseStamped,
                                                    queue_size=10)
        self.cv_bridge = CvBridge()

    def plan_grasp(self, image, mask, camera_info):
        """Grasp planner subscriber handler.

        Parameters
        ---------
        image: sensor_msgs.msg.Image
            ROS Image msg of depth image for grasp planner.
        mask: sensor_msgs.msg.Image
            ROS Image msg of segmask for grasp planner.
        camera_info: sensor_msgs.msg.CameraInfo
            ROS CameraInfo msg for grasp planner.
        """
        rospy.loginfo("Setting Camera Intrinsics")
        grasp_planning_start_time = timer()
        camera_intr = CameraIntrinsics(camera_info.header.frame_id,
                                       camera_info.K[0], camera_info.K[4],
                                       camera_info.K[2], camera_info.K[5],
                                       camera_info.K[1], camera_info.height,
                                       camera_info.width)

        rospy.loginfo("Planning Grasp")
        try:
            raw_depth = self.cv_bridge.imgmsg_to_cv2(
                image, desired_encoding="passthrough")
            raw_mask = self.cv_bridge.imgmsg_to_cv2(
                mask, desired_encoding="passthrough")
        except CvBridgeError as cv_err:
            rospy.logerr(cv_err)
        depth_im = DepthImage(raw_depth, frame=camera_intr.frame)
        segmask = BinaryImage(raw_mask, frame=camera_intr.frame)
        color_im = ColorImage(np.zeros(raw_depth.shape[:2] + (3, ),
                                       dtype=np.uint8),
                              encoding="rgb8",
                              frame=depth_im.frame)

        # Inpaint image.
        depth_im = depth_im.inpaint(
            rescale_factor=self.cfg["inpaint_rescale_factor"])

        # Aggregate color and depth images into a single
        # BerkeleyAutomation/autolab_core `RgbdImage`.
        rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)

        # Create an `RgbdImageState` with `RgbdImage` and `CameraIntrinsics`.
        rgbd_state = RgbdImageState(rgbd_im, camera_intr, segmask=segmask)

        # Execute policy.
        try:
            # Execute the policy"s action.
            grasp = grasping_policy(rgbd_state)

            # Create header
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = camera_intr.frame

            # Create `GQCNNGrasp` return msg and populate it.
            gqcnn_grasp = GQCNNGrasp()
            gqcnn_grasp.header = header
            gqcnn_grasp.q_value = grasp.q_value
            gqcnn_grasp.pose = grasp.grasp.pose().pose_msg

            # Set grasp type
            if isinstance(grasp.grasp, Grasp2D):
                gqcnn_grasp.grasp_type = GQCNNGrasp.PARALLEL_JAW
            elif isinstance(grasp.grasp, SuctionPoint2D):
                gqcnn_grasp.grasp_type = GQCNNGrasp.SUCTION

            # Store grasp representation in image space.
            gqcnn_grasp.center = grasp.grasp.center.data
            gqcnn_grasp.angle = grasp.grasp.angle
            gqcnn_grasp.depth = grasp.grasp.depth

            # Create and publish the pose alone for easy visualization of grasp
            # pose in Rviz.
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = gqcnn_grasp.pose
            self.grasp_pose_publisher.publish(pose_stamped)

            # Log time
            gqcnn_grasp.plan_time = timer() - grasp_planning_start_time
            self.grasp_publisher.publish(gqcnn_grasp)
            rospy.loginfo("Total grasp planning time: {:.3f} secs".format(
                gqcnn_grasp.plan_time))

        except NoValidGraspsException:
            rospy.logerr((
                "While executing policy, no valid grasps were found! Aborting Policy!"
            ))


if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("Grasp_Sampler_Server")

    # Get configs.
    model_config = json.load(open(os.path.join("/model", "config.json"), "r"))
    gqcnn_config = model_config["gqcnn"]
    gripper_mode = gqcnn_config["gripper_mode"]

    # Set config.
    if gripper_mode.lower() == GripperMode.PARALLEL_JAW:
        config_filename = "/root/gqcnn/cfg/examples/fc_gqcnn_pj.yaml"
    else:
        config_filename = "/root/gqcnn/cfg/examples/fc_gqcnn_suction.yaml"

    # Read config.
    cfg = YamlConfig(config_filename)
    policy_cfg = cfg["policy"]
    policy_cfg["metric"]["gqcnn_model"] = "/model"

    # Create grasping policy
    rospy.loginfo("Creating Grasping Policy")
    if gripper_mode.lower() == GripperMode.SUCTION:
        grasping_policy = FullyConvolutionalGraspingPolicySuction(policy_cfg)
    else:
        grasping_policy = FullyConvolutionalGraspingPolicyParallelJaw(
            policy_cfg)

    # Create a grasp planner.
    grasp_planner = GraspPlanner(cfg, grasping_policy)

    # Initialize the ROS subscribers and sync camera info with image.
    grasp_image_subscriber = message_filters.Subscriber("~image", Image)
    grasp_mask_subscriber = message_filters.Subscriber("~mask", Image)
    grasp_camera_info_subscriber = message_filters.Subscriber(
        "~camera_info", CameraInfo)
    time_sync = message_filters.TimeSynchronizer([
        grasp_image_subscriber, grasp_mask_subscriber,
        grasp_camera_info_subscriber
    ], 10)
    time_sync.registerCallback(grasp_planner.plan_grasp)
    rospy.loginfo("Grasping Policy Initialized")

    # Spin forever.
    rospy.spin()
