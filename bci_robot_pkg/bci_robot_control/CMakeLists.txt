cmake_minimum_required(VERSION 2.8.3)
project(bci_robot_control)


find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    diagnostic_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} src/bci_robot_control.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

