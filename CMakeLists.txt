cmake_minimum_required(VERSION 3.0.2)
project(hno_vio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DROS_AVAILABLE=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
  message_filters
  tf
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS include thirdparty/openvins_core/src
  LIBRARIES hno_vio_lib openvins_core_minimal
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs cv_bridge image_transport message_filters tf
)

include_directories(
  include
  thirdparty/openvins_core/src
  ${catkin_INCLUDE_DIRS}
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
target_link_libraries(openvins_core_minimal ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_library(hno_vio_lib
  src/HNOPropagator.cpp
  src/HNOUpdater.cpp
  src/HNOFeature.cpp
  src/HNOInitializer.cpp
  src/HNOManager.cpp
)
target_link_libraries(hno_vio_lib openvins_core_minimal ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(run_hno_vio src/run_hno_vio.cpp)
target_link_libraries(run_hno_vio hno_vio_lib openvins_core_minimal ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS openvins_core_minimal hno_vio_lib run_hno_vio
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config ground_truth thirdparty
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
