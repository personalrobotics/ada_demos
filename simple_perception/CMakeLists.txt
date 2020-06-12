cmake_minimum_required(VERSION 2.8.12)
project(simple_perception)

add_executable(simple_perception
  src/main.cpp)

target_link_libraries(simple_perception
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libada)

install(TARGETS simple_perception RUNTIME DESTINATION bin)
