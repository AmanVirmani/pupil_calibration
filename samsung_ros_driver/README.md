# samsung_ros_driver
ROS bindings for Samsung DVS camera

## THIS IS A PROPRIETARY SOFTWARE, DO NOT DISTRIBUTE!

## HOW TO USE
1) *roslaunch samsung_ros_driver samsung_publisher.launch*    # - to launch the driver
2) *rostopic list*    # - to see the list of published topics
3) *rosbag record -a*   # - record all topics in a .bag file ([tutorial](http://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data))

## Visualize
- 3D visualizer is in [better-flow](https://github.com/better-flow/better-flow) project, in `samsung_camera_support` branch.
  - Afrer cloning better-flow, run `git checkout samsung_camera_support`, then follow inistructions on how to build.

## Installation
- Clone, set up and build [cognifli](https://github.com/ncos/cognifli), also see instructions [here](https://github.com/better-flow/better-flow)
  - Install ROS, run `INSTALL.py`, then `catkin_make` as described
- Go to cognifli/src and run:
  - git clone [https://github.com/ncos/samsung_ros_wrapper](https://github.com/ncos/samsung_ros_wrapper)
  - git clone [https://github.com/ncos/samsung_ros_driver](https://github.com/ncos/samsung_ros_driver)
- Build *cognifli* again

