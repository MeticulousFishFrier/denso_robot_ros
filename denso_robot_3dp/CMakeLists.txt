cmake_minimum_required(VERSION 3.0.2)
project(denso_robot_3dp)


find_package(catkin REQUIRED
    moveit_ros_planning_interface
    moveit_visual_tools
    roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost COMPONENTS system REQUIRED)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp)

target_link_libraries(denso_robot_3dp ${catkin_LIBRARIES} ${Boost_LIBRARIES} )  