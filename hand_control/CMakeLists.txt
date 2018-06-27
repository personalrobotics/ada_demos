cmake_minimum_required(VERSION 2.8.12)
project(hand_control)

add_executable(hand_control
  src/main.cpp)

target_link_libraries(hand_control
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libada)

install(TARGETS hand_control RUNTIME DESTINATION bin)
