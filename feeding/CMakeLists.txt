cmake_minimum_required(VERSION 2.8.12)
project(feeding)


find_package(OpenCV REQUIRED)

add_executable(feeding
  src/main.cpp
  src/defaultmain.cpp
  src/studymain.cpp
  src/demomain.cpp
  src/bltmain.cpp
  src/acquisitionmain.cpp
  src/acquisitiontiltedmain.cpp
  src/FeedingDemo.cpp
  src/AdaMover.cpp
  src/FTThresholdHelper.cpp
  src/Perception.cpp
  src/Workspace.cpp
  src/util.cpp
  src/PerceptionServoClient.cpp
)

include_directories(include)

target_link_libraries(feeding
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  ${rewd_controllers_LIBRARIES}
  ${tf_conversions_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  libada)

install(TARGETS feeding RUNTIME DESTINATION bin)
