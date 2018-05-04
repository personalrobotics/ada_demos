cmake_minimum_required(VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${project_name})

add_executable(${project_name}
  src/main.cpp
)

target_link_libraries(${project_name}
  PUBLIC
    ${DART_LIBRARIES}
    ${aikido_LIBRARIES}
    ${Boost_LIBRARIES}
    libada
)

install(TARGETS ${project_name} RUNTIME DESTINATION bin)
