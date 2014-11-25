cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(Fuzzylite_DIR $ENV{MY_MODULE_PATH})

find_package(Fuzzylite REQUIRED)

if (Fuzzylite_FOUND)
  message(STATUS "Fuzzylite found. Libraries ${Fuzzylite_LIBRARIES} in ${Fuzzylite_LIBRARY_DIRS}. Include directory is ${Fuzzylite_INCLUDE_DIRS}.")
else (Fuzzylite_FOUND)
  message(ERROR "Fuzzylite not found.")
endif (Fuzzylite_FOUND)

set(Yamlcpp_DIR $ENV{MY_MODULE_PATH})

find_package(Yamlcpp REQUIRED)

if (Yamlcpp_FOUND)
  message(STATUS "Yamlcpp found. Libraries ${Yamlcpp_LIBRARIES} in ${Yamlcpp_LIBRARY_DIRS}. Header directory is ${Yamlcpp_INCLUDE_DIRS}.")
else (Yamlcpp_FOUND)
  message(ERROR "Yamlcpp not found.")
endif (Yamlcpp_FOUND)

set(Stage_DIR $ENV{MY_MODULE_PATH})
find_package(Stage REQUIRED)

if (Stage_FOUND)
  message(STATUS "Stage found. Libraries ${Stage_LIBRARIES} in ${Stage_LIBRARY_DIRS}. Header directories are ${Stage_INCLUDE_DIRS}")
else (Stage_FOUND)
  message(ERROR "Stage library not found.")
endif (Stage_FOUND)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add_executable( obst_avoid src/player_obst_avoid.cpp )
#target_link_libraries( obst_avoid fuzzy_oa ${Fuzzylite_LIBRARIES})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(fuzzy_oa SHARED src/robo_vel.cpp)
target_link_libraries( fuzzy_oa ${Fuzzylite_LIBRARIES} )

rosbuild_add_library( marker_details SHARED src/marker_details.cpp)
target_link_libraries( marker_details ${Yamlcpp_LIBRARIES} )

rosbuild_add_executable( player_obst_avoid src/player_obst_avoid.cpp )
target_link_libraries( player_obst_avoid fuzzy_oa )

rosbuild_add_executable( warehouse_marker_finder src/warehouse_marker_finder.cpp )
target_link_libraries( warehouse_marker_finder ${Stage_LIBRARIES} marker_details )
