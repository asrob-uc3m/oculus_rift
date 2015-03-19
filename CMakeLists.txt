cmake_minimum_required(VERSION 2.8.3)
project(rqt_servo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rospy std_msgs tf)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
    INCLUDE_DIRS src
    CATKIN_DEPENDS rospy
    DEPENDS
)

#############
## Install ##
#############

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_servo
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
