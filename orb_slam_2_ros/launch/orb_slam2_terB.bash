sudo apt install -y docker.io
sudo docker pull debbieliang/ros_open:posestamped
sudo docker run --network host --rm -it debbieliang/ros_open:posestamped /bin/bash -c "cd /orb_slam_2_ros/build && source devel/setup.bash && cd .. && roslaunch orb_slam2_ros orb_slam2_r200_mono.launch"
