cmake_minimum_required(VERSION 2.8)
project(ecto_image_pipeline)
find_package(catkin REQUIRED)
if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package(INCLUDE_DIRS include
               LIBRARIES ecto_image_pipeline
)
else()
catkin_stack()
catkin_project(ecto_image_pipeline
               INCLUDE_DIRS include
               LIBRARIES ecto_image_pipeline
)
endif()

catkin_python_setup()

find_package(ecto REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
install(DIRECTORY ${ecto_image_pipeline_SOURCE_DIR}/include/
        DESTINATION include
        COMPONENT main
)

include_directories(SYSTEM
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_subdirectory(cells)
add_subdirectory(src)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
