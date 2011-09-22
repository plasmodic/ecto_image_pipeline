cmake_minimum_required(VERSION 2.8)
project(my_project)

find_package(image_pipeline 0.1.0 REQUIRED)

include_directories(${image_pipeline_INCLUDE_DIRS})

add_executable(my_thing main.cpp)
target_link_libraries(my_thing ${image_pipeline_LIBRARIES})
