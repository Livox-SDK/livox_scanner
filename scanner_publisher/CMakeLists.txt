cmake_minimum_required(VERSION 2.8.3)
project(scanner_publisher)
add_definitions(-std=c++11)

# find packages
find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
)
add_definitions(${PCL_DEFINITIONS})

find_package(PCL REQUIRED)
find_package(PkgConfig)
find_package(Boost REQUIRED COMPONENTS system)
pkg_check_modules(APR apr-1) 
if (APR_FOUND) 
	message(${APR_INCLUDE_DIRS})
    message(${APR_LIBRARIES}) 
endif (APR_FOUND)

# add catkin instructions
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs tf
    DEPENDS system_lib
)

# for ROS
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Other includes
include_directories(
    ${APR_INCLUDE_DIRS}               
    ${PCL_INCLUDE_DIRS}
    ${PCL_LIBRARY_DIRS}
    /usr/local/include
)

# set source list
set(SOURCE_LIST src/pointcloud_publisher.cpp)

# link and make
link_directories(${ROS_CATKIN_WORKSPACE_LIBRARY_DIR} /usr/local/lib)
add_executable(${PROJECT_NAME}_node ${SOURCE_LIST})
target_link_libraries(
    ${PROJECT_NAME}_node
    liblivox_sdk_static.a
    ${APR_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )
