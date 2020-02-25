cmake_minimum_required(VERSION 2.8.12)
project(kevins_garbage)

add_executable(kevins_garbage
  src/main.cpp
  src/FTThresholdHelper.cpp)

include_directories(include)

target_link_libraries(kevins_garbage
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ada_demos_LIBRARIES}
  ${rewd_controllers_LIBRARIES}
  libada)

install(TARGETS kevins_garbage RUNTIME DESTINATION bin)
