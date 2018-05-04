cmake_minimum_required(VERSION 2.8.12)
project(feeding)

add_executable(feeding
  src/main.cpp)

target_link_libraries(feeding
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libada)

install(TARGETS feeding RUNTIME DESTINATION bin)
