cmake_minimum_required(VERSION 2.8.12)
project(feeding)

IF (rewd_controllers_FOUND)
  message(STATUS "rewd controllers found!")
  add_definitions(-LREWD_CONTROLLERS_FOUND)
ELSE()
message("rewd controllers not found!")
ENDIF()

#==============================================================================
# Required Dependencies
#
find_package(Boost REQUIRED COMPONENTS program_options)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag REQUIRED)

find_package(DART 6.6.0 REQUIRED
  COMPONENTS optimizer-nlopt utils
  CONFIG
)

include_directories(include)

add_executable(feeding
  scripts/main.cpp
  scripts/demo.cpp
  scripts/spanetDemo.cpp

  scripts/kinovaScoopDemo.cpp
  scripts/ScoopDemo.cpp
  
  scripts/pushingDemo.cpp

  # scripts/spanetDataCollection.cpp
  # scripts/acquisition.cpp
  # scripts/bite_location_detector.cpp
  # src/DataCollector.cpp
  src/AcquisitionAction.cpp
  src/DataCollector.cpp
  src/FoodItem.cpp
  src/FeedingDemo.cpp
  src/FTThresholdHelper.cpp
  src/Workspace.cpp
  src/util.cpp
  src/action/Grab.cpp
  src/action/DetectAndMoveAboveFood.cpp
  src/action/FeedFoodToPerson.cpp
  src/action/MoveAbove.cpp
  src/action/MoveAboveFood.cpp
  src/action/MoveAboveForque.cpp
  src/action/MoveAbovePlate.cpp
  src/action/MoveDirectlyToPerson.cpp
  src/action/MoveInFrontOfPerson.cpp
  src/action/MoveInto.cpp
  src/action/MoveOutOf.cpp
  src/action/MoveTowardsPerson.cpp
  src/action/PickUpFork.cpp
  src/action/PutDownFork.cpp
  src/action/Skewer.cpp

  src/action/KinovaScoop.cpp
  src/action/Scoop.cpp

  src/action/Pushing.cpp

  src/perception/Perception.cpp
  src/perception/PerceptionServoClient.cpp
  src/ranker/ShortestDistanceRanker.cpp
  src/ranker/SuccessRateRanker.cpp
  src/ranker/TargetFoodRanker.cpp
)

include_directories(include $(rosbag_INCLUDE_DIRS))

target_link_libraries(feeding
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  ${tf_conversions_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${rosbag_LIBRARIES}
  libada)

IF (rewd_controllers_FOUND)
  target_link_libraries(feeding ${rewd_controllers_LIBRARIES})
ENDIF()

install(TARGETS feeding RUNTIME DESTINATION bin)
