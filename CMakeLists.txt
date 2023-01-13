cmake_minimum_required(VERSION 3.16.3)
project(lexicographic_planning)

# debug mdoe.
SET(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_CXX_STANDARD 14)
# This makes C++14 a requirement,
# and prevents a "decay" to C++98 when the compiler does not support C++14.
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
# This disables the use of compiler-specific extensions.
# For example, by default, CMake passes -std=gnu++14 to GCC on Linux.
# We want to build with -std=c++14. 
SET(CMAKE_CXX_EXTENSIONS OFF)

ADD_DEFINITIONS(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rospy
    pcl_ros
    pcl_conversions
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    pluginlib
    # nav_core
)

find_package(PCL 1.13 REQUIRED)
SET(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.13/pcl)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    DEPENDS PCL
    CATKIN_DEPENDS 
    geometry_msgs 
    std_msgs
    nav_msgs
    visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_pathPlanning src/main.cpp src/pathPlanning.cpp src/rollout_generator.cpp)
target_link_libraries(${PROJECT_NAME}_pathPlanning ${catkin_LIBRARIES} ${PCL_LIBRARIES})

