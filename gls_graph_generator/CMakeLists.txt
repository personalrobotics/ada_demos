cmake_minimum_required(VERSION 2.8.12)
project(gls)

find_package(gls REQUIRED)

add_executable(graphGenerator
  src/graphGenerator.cpp
  src/configurations.cpp)
target_link_libraries(graphGenerator
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  ${gls_LIBRARIES}
  libada
  feeding
  )
install(TARGETS graphGenerator RUNTIME DESTINATION bin)

add_executable(viz
  src/viz.cpp)
target_link_libraries(viz
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  ${gls_LIBRARIES}
  libada
  feeding
  )
install(TARGETS viz RUNTIME DESTINATION bin)