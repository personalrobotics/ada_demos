cmake_minimum_required(VERSION 2.8.12)

add_executable(simple_perception
  src/main.cpp)

target_link_libraries(simple_perception
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  ${tf_conversions_LIBRARIES}
  libada)

install(TARGETS simple_perception RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
