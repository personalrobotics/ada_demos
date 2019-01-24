cmake_minimum_required(VERSION 2.8.12)
project(cameraCalibration)

find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)

add_executable(cameraCalibration
  src/main.cpp
  src/util.cpp
  src/Perception.cpp
)

include_directories(include)

target_link_libraries(cameraCalibration
  ${catkin_LIBRARIES}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_geometry_LIBRARIES}
  libada)

install(TARGETS cameraCalibration RUNTIME DESTINATION bin)
