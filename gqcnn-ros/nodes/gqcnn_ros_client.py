#!/usr/bin/env python
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

Displays robust grasps planned using a FC-GQ-CNN-based policy on a set of saved
RGB-D images.

Author
------
Mike Danielczuk, Jeff Mahler
"""

import argparse
import numpy as np
import os
import rospy
import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
from timeit import default_timer as timer

from autolab_core import Logger, Point, CameraIntrinsics, DepthImage, BinaryImage
from visualization import Visualizer2D as vis2d

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Image
from gqcnn_ros.msg import GQCNNGrasp

# Set up logger.
logger = Logger.get_logger("examples/policy_ros.py")


class GraspProcessor(object):
    def __init__(self,
                 depth_im,
                 camera_intr,
                 gripper_width=0.05,
                 vis_grasp=False):
        self.depth_im = depth_im
        self.camera_intr = camera_intr
        self.gripper_width = gripper_width
        self.vis_grasp = vis_grasp

        self.cur_q_val = None
        self.grasp_req_times = []
        self.grasp_plan_times = []

    @property
    def request_time_statistics(self):
        return len(self.grasp_req_times), np.mean(
            self.grasp_req_times), np.std(self.grasp_req_times)

    @property
    def planning_time_statistics(self):
        return len(self.grasp_plan_times), np.mean(
            self.grasp_plan_times), np.std(self.grasp_plan_times)

    def record_request_start(self):
        self.grasp_start_time = timer()

    def record_request_end(self):
        self.grasp_req_times.append(timer() - self.grasp_start_time)

    def record_plan_time(self, plan_time):
        self.grasp_plan_times.append(plan_time)

    def process(self, grasp):
        self.record_request_end()
        self.record_plan_time(grasp.plan_time)
        if self.vis_grasp:
            vis2d.figure(size=(5, 5))
            vis2d.imshow(self.depth_im, vmin=0.6, vmax=0.9)
            color = plt.get_cmap('hsv')(0.3 * grasp.q_value)[:-1]

            center_px = grasp.center
            center = self.camera_intr.deproject_pixel(
                grasp.depth,
                Point(np.array(grasp.center), frame=self.camera_intr.frame))
            if grasp.grasp_type == GQCNNGrasp.SUCTION:
                plt.scatter(*center_px, color=color, marker=".", s=100)

            else:
                plt.plot(*center_px, c=color, marker="+", mew=5, ms=2)
                axis = np.array([np.cos(grasp.angle), np.sin(grasp.angle), 0])
                end_points = [
                    center + Point(self.gripper_width / 2 * axis,
                                   frame=self.camera_intr.frame),
                    center - Point(self.gripper_width / 2 * axis,
                                   frame=self.camera_intr.frame)
                ]

                proj_end_points = [
                    self.camera_intr.project(end_points[0]).data,
                    self.camera_intr.project(end_points[1]).data
                ]
                proj_end_points_arrow = [
                    proj_end_points[0] - 8 * axis[:2],
                    proj_end_points[1] + 8 * axis[:2]
                ]
                vis2d.plot(
                    [proj_end_points_arrow[0][0], proj_end_points_arrow[1][0]],
                    [proj_end_points_arrow[0][1], proj_end_points_arrow[1][1]],
                    color=color,
                    linewidth=1,
                    linestyle="--")

                alpha = 4
                jaw_dir = 6 * np.array([axis[1], -axis[0]])

                # plot first jaw
                plt.arrow(proj_end_points_arrow[0][0],
                          proj_end_points_arrow[0][1],
                          alpha * axis[0],
                          alpha * axis[1],
                          width=2,
                          head_width=6,
                          head_length=4,
                          fc=color,
                          ec=color)
                jaw_line1 = np.c_[proj_end_points[0] + jaw_dir,
                                  proj_end_points[0] - jaw_dir].T
                plt.plot(jaw_line1[:, 0],
                         jaw_line1[:, 1],
                         linewidth=2,
                         c=color)

                # plot second jaw
                plt.arrow(proj_end_points_arrow[1][0],
                          proj_end_points_arrow[1][1],
                          -alpha * axis[0],
                          -alpha * axis[1],
                          width=2,
                          head_width=6,
                          head_length=4,
                          fc=color,
                          ec=color)
                jaw_line2 = np.c_[proj_end_points[1] + jaw_dir,
                                  proj_end_points[1] - jaw_dir].T
                plt.plot(jaw_line2[:, 0],
                         jaw_line2[:, 1],
                         linewidth=2,
                         c=color)

            plt.title("Planned grasp on depth (Q=%.3f)" % (grasp.q_value))
            plt.savefig(
                os.path.join(os.path.dirname(os.path.realpath(__file__)), "..",
                             "grasp.png"))
            plt.close()


if __name__ == "__main__":
    # Parse args.
    parser = argparse.ArgumentParser(
        description="Run a gqcnn ROS grasping policy on an example image")
    parser.add_argument("--depth_image",
                        type=str,
                        default=None,
                        help="path to depth image .npy file")
    parser.add_argument("--segmask",
                        type=str,
                        default=None,
                        help="path to segmask .png file")
    parser.add_argument("--camera_intr",
                        type=str,
                        default=None,
                        help="path to the camera intrinsics")
    parser.add_argument("--namespace",
                        type=str,
                        default="gqcnn",
                        help="namespace of the ROS grasp planner")
    parser.add_argument("--vis_grasp",
                        action="store_true",
                        help="whether or not to visualize the grasp")
    parser.add_argument("--num_pubs",
                        type=int,
                        default=100,
                        help="how many times to publish the message")
    parser.add_argument("--pub_rate",
                        type=float,
                        default=0.5,
                        help="publish rate for image")
    args = parser.parse_args(rospy.myargv()[1:])
    depth_im_filename = args.depth_image
    segmask_filename = args.segmask
    camera_intr_filename = args.camera_intr
    namespace = args.namespace
    vis_grasp = args.vis_grasp
    num_pubs = args.num_pubs
    pub_rate = args.pub_rate

    # Initialize the ROS node.
    rospy.init_node("gqcnn_ros_client")

    # Setup filenames.
    if depth_im_filename is None:
        depth_im_filename = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "..",
            "data/examples/phoxi_clutter/depth_0.npy")
    if camera_intr_filename is None:
        camera_intr_filename = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "..",
            "data/calib/phoxi.intr")

    # Set up sensor.
    camera_intr = CameraIntrinsics.load(camera_intr_filename)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = camera_intr.frame
    camera_info_msg = CameraInfo()
    camera_info_msg.header = header
    camera_info_msg.K = np.array([
        camera_intr.fx, camera_intr.skew, camera_intr.cx, 0.0, camera_intr.fy,
        camera_intr.cy, 0.0, 0.0, 1.0
    ])
    cam_info_pub = rospy.Publisher(rospy.resolve_name("~camera_info",
                                                      "/gqcnn"),
                                   CameraInfo,
                                   queue_size=10)

    # Read image and set up publisher
    depth_im = DepthImage.open(depth_im_filename, frame=camera_intr.frame)
    depth_pub = rospy.Publisher(rospy.resolve_name("~image_raw", "/gqcnn"),
                                Image,
                                queue_size=10)
    depth_im_msg = depth_im.rosmsg
    depth_im_msg.header = header

    if segmask_filename is not None:
        segmask = BinaryImage.open(segmask_filename, frame=depth_im.frame)
    else:
        segmask = BinaryImage(np.iinfo(np.uint8).max *
                              np.ones(depth_im.shape).astype(np.uint8),
                              frame=depth_im.frame)
    seg_pub = rospy.Publisher(rospy.resolve_name("~mask_raw", "/gqcnn"),
                              Image,
                              queue_size=10)
    segmask_msg = segmask.rosmsg
    segmask_msg.header = header

    # Set up subscribers and processor
    grasp_processor = GraspProcessor(depth_im,
                                     camera_intr,
                                     vis_grasp=vis_grasp)
    grasp_sub = rospy.Subscriber(rospy.resolve_name("~grasp", "/gqcnn"),
                                 GQCNNGrasp, grasp_processor.process)

    rate = rospy.Rate(pub_rate)  # 0.5 hz
    req_num = 0
    while not rospy.is_shutdown() and req_num < num_pubs:
        if grasp_processor.request_time_statistics[0] > 0:
            rospy.loginfo(
                "Request {:d} took {:.4f} s total ({:.4f} s planning)".format(
                    req_num - 1, grasp_processor.grasp_req_times[-1],
                    grasp_processor.grasp_plan_times[-1]))
        grasp_processor.record_request_start()
        cam_info_pub.publish(camera_info_msg)
        depth_pub.publish(depth_im_msg)
        seg_pub.publish(segmask_msg)
        req_num += 1
        rate.sleep()

    rospy.loginfo("Request Times ({:d} trials): {:.4f} +- {:.4f} s".format(
        *grasp_processor.request_time_statistics))
    rospy.loginfo("Planning Times ({:d} trials): {:.4f} +- {:.4f} s".format(
        *grasp_processor.planning_time_statistics))
