cmake_minimum_required(VERSION 2.8.12)
project(gelslight_experiments)

add_executable(gelslight_experiments
  src/main.cpp)

target_link_libraries(gelslight_experiments
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libada)

install(TARGETS gelslight_experiments RUNTIME DESTINATION bin)
