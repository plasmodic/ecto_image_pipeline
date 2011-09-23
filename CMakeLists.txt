cmake_minimum_required(VERSION 2.8)
project(image_pipeline)
set(image_pipeline_MAJOR_VERSION 0)
set(image_pipeline_MINOR_VERSION 1)
set(image_pipeline_PATCH_VERSION 0)
set(image_pipeline_SOVERSION 
  ${image_pipeline_MAJOR_VERSION}.${image_pipeline_MINOR_VERSION}
)
set(image_pipeline_VERSION 
  ${image_pipeline_MAJOR_VERSION}.${image_pipeline_MINOR_VERSION}.${image_pipeline_PATCH_VERSION}
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_definitions("-Wall -Werror")

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

add_subdirectory(src)

option(BUILD_ECTO_CELLS "Build ecto cells" ON)
if(BUILD_ECTO_CELLS)
  add_subdirectory(cells)
endif()

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

#these setup the lib to be used by others
include(cmake/install.cmake)
include(cmake/config.cmake)
