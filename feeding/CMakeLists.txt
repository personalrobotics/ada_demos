cmake_minimum_required(VERSION 2.8.12)
project(feeding)


add_executable(feeding
  src/main.cpp
  src/FeedingDemo.cpp
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
  libada)

install(TARGETS feeding RUNTIME DESTINATION bin)
