cmake_minimum_required(VERSION 2.8.3)
project(ndt_localization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp ndt_map sensor_msgs geometry_msgs laser_geometry message_filters
  velodyne_pointcloud tf_conversions
  )
# set(CMAKE_CXX_FLAGS "-std=c++11 -ggdb ${CMAKE_CXX_FLAGS}")
# find_package(OpenMP)

set (CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ndt_map roscpp sensor_msgs geometry_msgs laser_geometry message_filters velodyne_pointcloud tf_conversions
  DEPENDS
  LIBRARIES ${PROJECT_NAME}
)


include_directories(include
   ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/particle.cpp
  src/${PROJECT_NAME}/particle_filter.cpp
  src/${PROJECT_NAME}/particle_filter_HMT.cpp
  src/${PROJECT_NAME}/helper_functions.cpp
  src/${PROJECT_NAME}/ndt_histogram_2D.cpp
  src/${PROJECT_NAME}/3d_ndt_mcl.cpp
  src/${PROJECT_NAME}/3d_particle_filter.cpp
)

add_executable(ndt_mcl_localization src/ndt_mcl_localization.cpp)
add_executable(ndt_mcl_localization_filter src/ndt_mcl_localization_filter.cpp)
add_executable(ndt_map_publisher src/ndt_map_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(ndt_mcl_localization
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(ndt_mcl_localization_filter
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(ndt_map_publisher
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ndt_map_publisher ndt_mcl_localization_filter ndt_mcl_localization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
