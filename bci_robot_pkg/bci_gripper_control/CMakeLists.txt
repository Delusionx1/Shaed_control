cmake_minimum_required(VERSION 2.8.3)
project(bci_gripper_control)

#find_package(catkin REQUIRED COMPONENTS
#  robotiq_2f_gripper_control
#)

add_executable(bci_gripper_control src/bci_gripper_control.cpp)
target_link_libraries(bci_gripper_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(bci_gripper_control ${catkin_EXPORTED_TARGETS})
install(TARGETS bci_gripper_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(bci_gripper_control_test src/bci_gripper_control_test.cpp)
target_link_libraries(bci_gripper_control_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(bci_gripper_control_test ${catkin_EXPORTED_TARGETS})
install(TARGETS bci_gripper_control_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
