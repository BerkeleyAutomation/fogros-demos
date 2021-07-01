sudo apt install -y docker.io
sudo docker pull debbieliang/ros_open:posestamped 
sudo docker run --network host --rm -it debbieliang/ros_open:posestamped /bin/bash -c "./orb_slam_2_ros/build/devel/lib/orb_slam2_ros/orb_slam2_ros_pss"
