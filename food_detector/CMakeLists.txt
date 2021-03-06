cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

	list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
project(food_detector)

rosbuild_init()
#set(PCL_DIR "/opt/ros/electric/stacks/perception_pcl/pcl/share/pcl-1.1/")
#find_package(PCL 1.2 REQUIRED)
#find_package(FLANN 1.7.1 REQUIRED)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_LIBRARY_DIRS})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_library(${PROJECT_NAME} src/food_detector.cpp)

rosbuild_add_executable(${PROJECT_NAME} src/vfh_food_classifier.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/food_detector.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/food_detector.cpp)

#target_link_libraries(food_detector ${PROJECT_NAME})


find_package(HDF5)
if(HDF5_FOUND)

  #find_package(Flann)
  include_directories(${FLANN_INCLUDE_DIRS})

  include_directories(${HDF5_INCLUDE_DIR})

  add_executable(${PROJECT_NAME} src/vfh_food_classifier.cpp)
  target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} 
                                        ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} 
                                        )
endif(HDF5_FOUND)
