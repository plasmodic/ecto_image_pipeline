cmake_minimum_required(VERSION 2.8)
project(ecto_image_pipeline)
find_package(catkin REQUIRED)
catkin_stack()
catkin_project(ecto_image_pipeline
               INCLUDE_DIRS include
               LIBRARIES ecto_image_pipeline
               DEPENDS Boost Eigen OpenCV
)

find_package(ecto REQUIRED)
set_ecto_install_package_name(ecto_image_pipeline)

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

add_subdirectory(src)

option(BUILD_ECTO_CELLS "Build ecto cells" ON)
if(BUILD_ECTO_CELLS)
  find_package(ecto REQUIRED)

  catkin_python_setup()
  add_subdirectory(cells)
endif()

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

#catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
