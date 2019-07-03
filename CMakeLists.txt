cmake_minimum_required(VERSION 2.8.12)
project(ada_demos)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED
  COMPONENTS
    tf
    tf_conversions
    image_transport
    cv_bridge)
catkin_package(
  INCLUDE_DIRS feeding/include
  LIBRARIES feeding
  )

# ===============================================================================
# Dependencies
#

find_package(tf_conversions REQUIRED)
include_directories(${tf_conversions_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

find_package(DART 6.6.2 REQUIRED
  COMPONENTS optimizer-nlopt utils
  OPTIONAL_COMPONENTS utils-urdf # for 'perception' target
  CONFIG
)

include_directories(${DART_INCLUDE_DIRS})

find_package(aikido 0.3.0 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    io
    perception
    planner
    planner_kunzretimer
    planner_ompl
    planner_parabolic
    planner_vectorfield
    planner_kunzretimer
    robot
    rviz
    statespace
)
include_directories(${aikido_INCLUDE_DIRS})

find_package(libada REQUIRED)
include_directories(${libada_INCLUDE_DIRS})

find_package(magi REQUIRED)
include_directories(${magi_INCLUDE_DIRS})

find_package(rewd_controllers)
IF (rewd_controllers_FOUND)
  include_directories(${rewd_controllers_INCLUDE_DIRS})
ENDIF()

find_package(pr_assets REQUIRED)
include_directories(${pr_assets_INCLUDE_DIRS})

find_package(pr_tsr REQUIRED)
include_directories(${pr_tsr_INCLUDE_DIRS})

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# ===============================================================================
# Demos
#

# add_subdirectory(soda_grasp)
# add_subdirectory(simple_trajectories)
# add_subdirectory(simple_perception)
# add_subdirectory(hand_control)
add_subdirectory(feeding)
# add_subdirectory(cameraCalibration)

# ===============================================================================
# Formatting
#
include(ClangFormat)
clang_format_setup(VERSION 3.8)

if(CLANG_FORMAT_EXECUTABLE)
  file(GLOB_RECURSE ALL_SOURCE_FILES
    LIST_DIRECTORIES false
    soda_grasp/src/*.cpp
    simple_trajectories/src/*.cpp
    feeding/src/*.cpp
    feeding/include/feeding/*.hpp
  )

  clang_format_add_sources(${ALL_SOURCE_FILES})
  clang_format_add_targets()
endif()
