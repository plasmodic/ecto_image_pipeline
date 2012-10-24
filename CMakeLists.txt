cmake_minimum_required(VERSION 2.8)
project(ecto_image_pipeline)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED opencv_candidate)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package(DEPENDS Boost Eigen OpenCV opencv_candidate PCL
               INCLUDE_DIRS include
               LIBRARIES ecto_image_pipeline
)
else()
catkin_stack()
catkin_project(ecto_image_pipeline
               DEPENDS Boost Eigen OpenCV PCL
               INCLUDE_DIRS include
               LIBRARIES ecto_image_pipeline
)
endif()

catkin_python_setup()

find_package(ecto REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

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
