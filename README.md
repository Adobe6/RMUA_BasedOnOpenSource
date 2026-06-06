项目结构最外层是basic_dev/

编译顺序：
- cd basic_dev
- ./src/livox_ros_driver2/build.sh ROS1
- catkin_make -DCATKIN_WHITELIST_PACKAGES="airsim_ros" 
- catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio" 
- catkin_make -DCATKIN_WHITELIST_PACKAGES="path_sender"
- catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j14
