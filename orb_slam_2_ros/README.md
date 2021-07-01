# ORB-SLAM2
**ORB-SLAM2 Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).
The original implementation can be found [here](https://github.com/raulmur/ORB_SLAM2.git). <br />
For the original README, please see README_original.md in this repo. For the original orb_slam2_ros, please see this [repo](https://github.com/appliedAI-Initiative/orb_slam_2_ros).
For the 'Image.msg' publisher, please refer to this [repo](https://github.com/amc-nu/RosImageFolderPublisher).
For the `PoseStamped.msg` subscriber, please refer to this [repo](https://gist.github.com/wjwwood/b67d07a171f89faa8939).

# Instructions for orb_slam2_ros in FogROS

## Docker image
Pull the docker image on an EC2 machine. The experiment is tested on AMI `ami-05829bd3e68bcd415` (North California), with instance type `c4.8xlarge` and storage around 48GB.
```
docker pull debbieliang/ros_open:posestamped 
```
Run on four separate terminals
```
sudo docker run --network host --rm -it debbieliang/ros_open:posestamped
``` 

## Terminal A
```
roscore
```

## Terminal B
Subscribe `Image.msg`, run the `ORB_SLAM2` algorithm, and publish `PoseStamped.msg`.
Run 
```
cd /orb_slam_2_ros/build
source devel/setup.bash 
cd .. 
roslaunch orb_slam2_ros orb_slam2_r200_mono.launch 
```
If you changed any code, run 

```
cd /orb_slam_2_ros/build
cmake .. -DROS_BUILD_TYPE=Release
make -j
source devel/setup.bash 
cd .. 
roslaunch orb_slam2_ros orb_slam2_r200_mono.launch 
```

## Terminal C 
Subscribe `PoseStamped.msg` from topic `/orb_slam2_mono/pose`
```
./orb_slam_2_ros/build/devel/lib/orb_slam2_ros/orb_slam2_ros_pss
```

## Terminal D 
Publish the png images in `/rgbd_dataset_freiburg1_xyz/rgb` through topic `/camera/rgb/image_raw`
```
cd /RosImageFolderPublisher
source devel/setup.bash
cd src
rosrun image_folder_publisher image_folder_publisher.py _image_folder:=./../../rgbd_dataset_freiburg1_xyz/rgb _topic_name=/camera/image_raw
```

## Expected prints on terminals
### Terminal A
```... logging to /home/ubuntu/.ros/log/31af6a50-ad06-11eb-9342-4fd7652998b2/roslaunch-ip-172-31-0-70-1949.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ip-172-31-0-70:44875/
ros_comm version 1.15.9


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.9

NODES

auto-starting new master
process[master]: started with pid [1974]
ROS_MASTER_URI=http://ip-172-31-0-70:11311/

setting /run_id to 31af6a50-ad06-11eb-9342-4fd7652998b2
process[rosout-1]: started with pid [1984]
started core service [/rosout]```
```
### Terminal B
```
... logging to /root/.ros/log/31af6a50-ad06-11eb-9342-4fd7652998b2/roslaunch-ip-172-31-0-70-52.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ip-172-31-0-70:40097/

SUMMARY
========

PARAMETERS
 * /orb_slam2_mono/ORBextractor/iniThFAST: 20
 * /orb_slam2_mono/ORBextractor/minThFAST: 7
 * /orb_slam2_mono/ORBextractor/nFeatures: 2000
 * /orb_slam2_mono/ORBextractor/nLevels: 8
 * /orb_slam2_mono/ORBextractor/scaleFactor: 1.2
 * /orb_slam2_mono/camera_cx: 311.43603515625
 * /orb_slam2_mono/camera_cy: 248.0950164794922
 * /orb_slam2_mono/camera_fps: 30
 * /orb_slam2_mono/camera_frame_id: camera_link
 * /orb_slam2_mono/camera_fx: 632.7927856445312
 * /orb_slam2_mono/camera_fy: 626.8605346679688
 * /orb_slam2_mono/camera_k1: -0.09097914397716522
 * /orb_slam2_mono/camera_k2: 0.06503549218177795
 * /orb_slam2_mono/camera_k3: 0.0
 * /orb_slam2_mono/camera_p1: 0.000849052332341671
 * /orb_slam2_mono/camera_p2: 0.001785792293958366
 * /orb_slam2_mono/camera_rgb_encoding: True
 * /orb_slam2_mono/load_calibration_from_cam: False
 * /orb_slam2_mono/load_map: False
 * /orb_slam2_mono/localize_only: False
 * /orb_slam2_mono/map_file: map.bin
 * /orb_slam2_mono/min_num_kf_in_map: 5
 * /orb_slam2_mono/pointcloud_frame_id: map
 * /orb_slam2_mono/publish_pointcloud: True
 * /orb_slam2_mono/publish_pose: True
 * /orb_slam2_mono/reset_map: False
 * /orb_slam2_mono/voc_file: /orb_slam_2_ros/o...
 * /rosdistro: noetic
 * /rosversion: 1.15.9

NODES
  /
    orb_slam2_mono (orb_slam2_ros/orb_slam2_ros_mono)

ROS_MASTER_URI=http://localhost:11311

process[orb_slam2_mono-1]: started with pid [60]

ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza.
This program comes with ABSOLUTELY NO WARRANTY;
This is free software, and you are welcome to redistribute it
under certain conditions. See LICENSE.txt.

OpenCV version : 4.2.0
Major version : 4
Minor version : 2
Subminor version : 0
Input sensor was set to: Monocular

Loading ORB Vocabulary.
Vocabulary loaded!


Camera Parameters: 
- fx: 632.793
- fy: 626.861
- cx: 311.436
- cy: 248.095
- k1: -0.0909791
- k2: 0.0650355
- p1: 0.000849052
- p2: 0.00178579
- fps: 30
- bf: 4.59149e-41
- color order: RGB (ignored if grayscale)

ORB Extractor Parameters: 
- Number of Features: 2000
- Scale Levels: 8
- Scale Factor: 1.2
- Initial Fast Threshold: 20
- Minimum Fast Threshold: 7
Enable localization only: false
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
Map point vector is empty!
New Map created with 118 points
[ WARN] [1620153702.356519172]: "camera_link" passed to lookupTransform argument target_frame does not exist. 
[ WARN] [1620153702.358269727]: "camera_link" passed to lookupTransform argument target_frame does not exist. 
[ WARN] [1620153703.273954453]: "camera_link" passed to lookupTransform argument target_frame does not exist.  
```
(The first `Map point vector is empty!` should appear after Terminal D starts publishing)

### Terminal C
```
[ INFO] [1620153702.358662571]: Received pose: 0x5616e6fbc8f0
[ INFO] [1620153703.274365240]: Received pose: 0x5616e6fbcef0
[ INFO] [1620153704.262915530]: Received pose: 0x5616e6fbd2c0
```
(Should appear after `New Map created with ... points` on terminal B)

### Terminal D 
```
[INFO] [1620153666.206704]: [image_folder_publisher] (topic_name) Publishing Images to topic  /camera/rgb/image_raw
[INFO] [1620153666.209049]: [image_folder_publisher] (publish_rate) Publish rate set to 1 hz
[INFO] [1620153666.210476]: [image_folder_publisher] (sort_files) Sort Files: True
[INFO] [1620153666.211829]: [image_folder_publisher] (frame_id) Frame ID set to  camera
[INFO] [1620153666.213169]: [image_folder_publisher] (loop) Loop  1 time(s) (set it -1 for infinite)
[INFO] [1620153666.218204]: [image_folder_publisher] Reading images from ./../../rgbd_dataset_freiburg1_xyz/rgb
[INFO] [1620153666.256244]: [image_folder_publisher] Published ./../../rgbd_dataset_freiburg1_xyz/rgb/1305031102.175304.png
[INFO] [1620153667.232531]: [image_folder_publisher] Published ./../../rgbd_dataset_freiburg1_xyz/rgb/1305031102.211214.png
[INFO] [1620153668.231347]: [image_folder_publisher] Published ./../../rgbd_dataset_freiburg1_xyz/rgb/1305031102.243211.png
```












