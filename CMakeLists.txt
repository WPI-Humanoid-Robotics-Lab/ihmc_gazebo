cmake_minimum_required(VERSION 2.8.3)
project(ihmc_gazebo)
set (CMAKE_CXX_STANDARD 11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

find_package(Boost REQUIRED system thread)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
 
file(COPY model/polaris_ranger_xp900_no_roll_cage DESTINATION $ENV{HOME}/.gazebo/models)
file(COPY meshes_unplugged DESTINATION $ENV{HOME}/.gazebo/models/ihmc_gazebo)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS roscpp gazebo_ros
  LIBRARIES TCPServer
)


add_library(TCPServer SHARED src/TCPServer.cc src/TCPConnection.cc  )
#add_executable(ServerTest ServerTest.cc)
#target_link_libraries(ServerTest ${Boost_LIBRARIES} pthread TCPServer)


add_library(DRCSimIHMCPlugin SHARED src/DRCSimIHMCPlugin.cpp)
target_link_libraries(DRCSimIHMCPlugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} TCPServer)

## Mark executables and/or libraries for installation
install(TARGETS DRCSimIHMCPlugin TCPServer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY yaml urdf worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
