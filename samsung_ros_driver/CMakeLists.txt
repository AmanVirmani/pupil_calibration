cmake_minimum_required(VERSION 2.8.3)
project(samsung_ros_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        samsung_event_msgs
        )

catkin_package(
  #INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs samsung_event_msgs
)


include_directories(
  #include/${PROJECT_NAME}
  cyusb_linux_1.0.4/include
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

add_executable(samsung_ros_publisher
    src/samsung_ros_publisher.cpp
)

target_link_libraries(
    samsung_ros_publisher
    ${catkin_LIBRARIES}
    pthread
    cyusb
)
