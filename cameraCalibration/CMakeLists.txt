cmake_minimum_required(VERSION 2.8.12)
project(cameraCalibration)

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
  libada)

install(TARGETS cameraCalibration RUNTIME DESTINATION bin)
