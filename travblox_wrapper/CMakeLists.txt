cmake_minimum_required(VERSION 3.5)
project(travblox_wrapper)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(voxblox_proto REQUIRED)
find_package(voxblox_ros REQUIRED)
find_package(voxblox REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add compile definitions
add_definitions(-DCOL_CHECK_METHOD=0)
add_definitions(-DEDGE_CHECK_METHOD=1)
add_definitions(-DRAY_CAST_METHOD=1)

# Verify the values of the definitions
get_directory_property(DirDefs COMPILE_DEFINITIONS) 
message("COMPILE DEFINITIONS: ${DirDefs}")

# Check for OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message("OpenMP enabled")
endif()

ament_auto_find_build_dependencies()

set(AMENT_DEPENDENCIES
    rclcpp
    std_srvs
    visualization_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    nav_msgs
    tf2_geometry_msgs
    glog
    voxblox
    voxblox_proto
    voxblox_msgs
    tf2
    tf2_ros
    pcl_ros
    pcl_conversions
    Eigen3
    grid_map_ros
    grid_map_msgs
)

set(OTHER_DEPENDENCIES
  glog::glog
  Eigen3::Eigen3
)

#############
# LIBRARIES #
#############

ament_auto_add_library(${PROJECT_NAME}
  src/params.cpp
  src/travblox_common.cpp
  src/travblox_main.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  ${AMENT_DEPENDENCIES}
)

target_link_libraries(${PROJECT_NAME} 
  ${OTHER_DEPENDENCIES}
)

# ament_auto_add_executable(background_processes_node
#   src/utils/background_processes_node.cpp
# )
# target_link_libraries(background_processes_node 
#   ${PROJECT_NAME}
# )

# ament_auto_add_executable(test_node
#   src/test/test_node.cpp
# )
# target_link_libraries(test_node 
#   ${PROJECT_NAME}
# )

ament_auto_package()

# install(
#   DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/
# )
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)
