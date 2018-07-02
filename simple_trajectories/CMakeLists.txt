cmake_minimum_required(VERSION 2.8.12)
project(simple_trajectories)

add_executable(simple_trajectories
  #src/main.cpp
  src/hammer.cpp)

target_link_libraries(simple_trajectories
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libada)

add_executable(load_trajectories
  src/load_trajectories.cpp)

target_link_libraries(load_trajectories
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libada)

install(TARGETS simple_trajectories RUNTIME DESTINATION bin)
install(TARGETS load_trajectories RUNTIME DESTINATION bin)
