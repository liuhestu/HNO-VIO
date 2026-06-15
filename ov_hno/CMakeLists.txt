cmake_minimum_required(VERSION 3.0.2)
project(ov_hno)

# Enable C++17 for structured bindings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  ov_core
  ov_init
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp ov_core ov_init
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/HNOPropagator.cpp
  src/HNOUpdater.cpp
  src/HNOFeature.cpp
  src/HNOInitializer.cpp
  src/HNOManager.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(run_subscribe_hno src/run_subscribe_hno.cpp)
target_link_libraries(run_subscribe_hno ${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_lib run_subscribe_hno
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
