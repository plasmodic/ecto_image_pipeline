cmake_minimum_required(VERSION 2.8)
project(ecto_image_pipeline)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED cmake_modules opencv_candidate)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL QUIET COMPONENTS common)
if(PCL_FOUND)
catkin_package(CATKIN_DEPENDS opencv_candidate
               DEPENDS Boost Eigen OpenCV PCL
               INCLUDE_DIRS include
               LIBRARIES ecto_image_pipeline
)
else()
catkin_package(CATKIN_DEPENDS opencv_candidate
               DEPENDS Boost Eigen OpenCV
               INCLUDE_DIRS include
               LIBRARIES ecto_image_pipeline
)
endif()

catkin_python_setup()

find_package(ecto REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
if(PCL_FOUND)
install(DIRECTORY ${ecto_image_pipeline_SOURCE_DIR}/include/ecto_image_pipeline/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        COMPONENT main
)
else()
install(DIRECTORY ${ecto_image_pipeline_SOURCE_DIR}/include/ecto_image_pipeline/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        COMPONENT main
        PATTERN *.h
        PATTERN *.hpp 
        PATTERN pcl EXCLUDE
)
endif()

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIRS}
)

add_subdirectory(cells)
add_subdirectory(src)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
