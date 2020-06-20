cmake_minimum_required(VERSION 2.8.3)
project(cross_calib)

#set(CMAKE_C_COMPILER "/usr/bin/clang-6.0")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++-6.0")

#set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
             cv_bridge
             geometry_msgs
             image_transport
             message_generation
             roscpp
             sensor_msgs
             visualization_msgs
             tf
             tf_conversions
             std_msgs
             vicon
             dvs_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs std_msgs geometry_msgs vicon dvs_msgs)

include_directories(.
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})


###########
## Build ##
###########


find_package(X11 REQUIRED)
add_executable(collect_frames collect.cpp)

target_link_libraries(collect_frames
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${X11_LIBRARIES}
)
