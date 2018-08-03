cmake_minimum_required(VERSION 2.8.12)
project(feeding)


find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)

add_executable(feeding
  src/main.cpp
  src/FeedingDemo.cpp
  src/AdaMover.cpp
  src/FTThresholdHelper.cpp
  src/Perception.cpp
  src/Workspace.cpp
  src/util.cpp
  src/PerceptionServoClient.cpp
  src/external/ParabolicRamp.cpp
  src/CheckerboardPerception.cpp
)

include_directories(include)

target_link_libraries(feeding
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  ${rewd_controllers_LIBRARIES}
  ${tf_conversions_LIBRARIES}
  ${OpenCV_LIBS}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_geometry_LIBRARIES}
  libada)

install(TARGETS feeding RUNTIME DESTINATION bin)
