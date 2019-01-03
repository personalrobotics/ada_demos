cmake_minimum_required(VERSION 2.8.12)
project(feeding)

IF (rewd_controllers_FOUND)
  message("rewd controllers found!")
  add_definitions(-LREWD_CONTROLLERS_FOUND)
ELSE()
message("rewd controllers not found!")
ENDIF()

#==============================================================================
# Required Dependencies
#
find_package(Boost REQUIRED COMPONENTS program_options)

find_package(OpenCV REQUIRED)

find_package(DART 6.6.0 REQUIRED
  COMPONENTS optimizer-nlopt utils
  CONFIG
)

include_directories(include)

add_executable(feeding
  scripts/main.cpp
  scripts/demo.cpp
  # scripts/bite_location_detector.cpp
  scripts/acquisition.cpp
  src/FeedingDemo.cpp
  src/FTThresholdHelper.cpp
  src/Perception.cpp
  src/Workspace.cpp
  src/util.cpp
  src/PerceptionServoClient.cpp
  
)

target_link_libraries(feeding
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  ${tf_conversions_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  libada)

IF (rewd_controllers_FOUND)
  target_link_libraries(feeding ${rewd_controllers_LIBRARIES})
ENDIF()

install(TARGETS feeding RUNTIME DESTINATION bin)
