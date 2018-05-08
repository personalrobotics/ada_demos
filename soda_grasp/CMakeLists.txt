cmake_minimum_required(VERSION 2.8.12)
project(soda_grasp)

add_executable(soda_grasp
  src/main.cpp)

target_link_libraries(soda_grasp
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libada)

install(TARGETS soda_grasp RUNTIME DESTINATION bin)
