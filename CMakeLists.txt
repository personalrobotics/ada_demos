cmake_minimum_required(VERSION 2.8.12)
project(ada_demos)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin COMPONENTS tf tf_conversions image_transport
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

find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})

find_package(aikido 0.0.2 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    io
    perception
    planner
    planner_ompl
    planner_parabolic
    planner_vectorfield
    planner_kinodynamic
    robot
    rviz
    statespace
)
include_directories(${aikido_INCLUDE_DIRS})

find_package(libada REQUIRED)
include_directories(${libada_INCLUDE_DIRS})

find_package(rewd_controllers)
IF (rewd_controllers_FOUND)
  include_directories(${rewd_controllers_INCLUDE_DIRS})
ENDIF()

find_package(pr_ordata REQUIRED)
include_directories(${pr_ordata_INCLUDE_DIRS})

find_package(pr_tsr REQUIRED)
include_directories(${pr_tsr_INCLUDE_DIRS})

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# ===============================================================================
# Demos
#

#add_subdirectory(soda_grasp)
add_subdirectory(simple_trajectories)
add_subdirectory(hand_control)
add_subdirectory(feeding)

# ===============================================================================
# Formatting
#

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format-3.8)

if(CLANG_FORMAT_EXECUTABLE)
  message(STATUS "Found clang-format.")

  file(GLOB_RECURSE ALL_SOURCE_FILES
    LIST_DIRECTORIES false
    soda_grasp/src/*.cpp
    simple_trajectories/src/*.cpp
    feeding/src/*.cpp
    feeding/include/feeding/*.hpp
  )
  list(LENGTH ALL_SOURCE_FILES NUM_SOURCE_FILES)

  add_custom_target(format
    COMMAND ${CMAKE_COMMAND} -E echo "Formatting ${NUM_SOURCE_FILES} files..."
    COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${ALL_SOURCE_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    DEPENDS ${CLANG_FORMAT_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_custom_target(check-format
    COMMAND ${CMAKE_COMMAND} -E echo "Checking code style of"
            "${NUM_SOURCE_FILES} files... "
    COMMAND ${CMAKE_SOURCE_DIR}/tools/check_format.sh
            ${CLANG_FORMAT_EXECUTABLE} ${ALL_SOURCE_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    DEPENDS ${CLANG_FORMAT_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
else()
  message(STATUS "Could not find clang-format. Please install "
      "clang-format to enable automatic code formatting."
  )
endif()
