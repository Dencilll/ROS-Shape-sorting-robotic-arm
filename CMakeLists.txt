cmake_minimum_required(VERSION 3.0.2)
project(shape_sorting_robot)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    visualization_msgs
    robot_state_publisher
)

catkin_package()

catkin_install_python(
    PROGRAMS scripts/sorting_controller.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
