cmake_minimum_required(VERSION 3.8)
project(hno_vio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_definitions(ROS_AVAILABLE=2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(CURL REQUIRED)

set(HNO_ROS_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  message_filters
  tf2
  tf2_ros
)

include_directories(
  include
  thirdparty/openvins_core/src
  ${EIGEN3_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(openvins_core_minimal
  thirdparty/openvins_core/src/feat/Feature.cpp
  thirdparty/openvins_core/src/feat/FeatureDatabase.cpp
  thirdparty/openvins_core/src/track/TrackBase.cpp
  thirdparty/openvins_core/src/track/TrackKLT.cpp
  thirdparty/openvins_core/src/utils/print.cpp
)
ament_target_dependencies(openvins_core_minimal ${HNO_ROS_DEPS})
target_link_libraries(openvins_core_minimal ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} CURL::libcurl)

add_library(hno_vio_lib
  src/Diagnostics.cpp
  src/observer/Propagator.cpp
  src/observer/Updater.cpp
  src/observer/ZuptUpdater.cpp
  src/frontend/FeatureManager.cpp
  src/frontend/StereoTriangulator.cpp
  src/frontend/LandmarkMap.cpp
  src/frontend/FeatureHealth.cpp
  src/Initializer.cpp
  src/pipeline/ImuBuffer.cpp
  src/pipeline/VioPipeline.cpp
  src/pipeline/OdomExport.cpp
  src/ros/GTMapping.cpp
  src/ros/RosPublisher.cpp
  src/ros/HnoVioNode.cpp
)
ament_target_dependencies(hno_vio_lib ${HNO_ROS_DEPS})
target_link_libraries(hno_vio_lib openvins_core_minimal ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} CURL::libcurl)

add_executable(run_hno_vio src/run_hno_vio.cpp)
ament_target_dependencies(run_hno_vio ${HNO_ROS_DEPS})
target_link_libraries(run_hno_vio hno_vio_lib openvins_core_minimal ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} CURL::libcurl)

install(TARGETS openvins_core_minimal hno_vio_lib run_hno_vio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config ground_truth
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY tools
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS tools/run_rtabmap/rtabmap_preprocess.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rtabmap_preprocess
)
install(PROGRAMS tools/run_rtabmap/export_optimized_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME export_optimized_odom
)
install(PROGRAMS tools/run_rtabmap/eval_and_analysis.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME eval_and_analysis
)

ament_export_include_directories(include thirdparty/openvins_core/src)
ament_export_libraries(hno_vio_lib openvins_core_minimal)
ament_export_dependencies(${HNO_ROS_DEPS})

ament_package()
