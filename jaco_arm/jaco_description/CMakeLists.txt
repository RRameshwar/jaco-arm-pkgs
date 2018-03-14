cmake_minimum_required(VERSION 2.8.3)
project(jaco_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS urdf 
   CATKIN_DEPENDS xacro
   gazebo_ros
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## install urdf files
install(DIRECTORY urdf 
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY meshes 
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(index_contact SHARED IndexContactPlugin.cc)
add_library(thumb_contact SHARED ThumbContactPlugin.cc)
add_library(middle_contact SHARED MiddleContactPlugin.cc)

target_link_libraries(index_contact ${GAZEBO_libraries})
target_link_libraries(thumb_contact ${GAZEBO_libraries})
target_link_libraries(middle_contact ${GAZEBO_libraries})
