find_package(ecto QUIET)
if(NOT ecto_FOUND)
  return()
endif()

find_package(ecto_opencv REQUIRED)
include_directories(SYSTEM ${ecto_opencv_INCLUDE_DIRS})

ectomodule(ecto_image_pipeline DESTINATION ecto_image_pipeline
                               INSTALL
   module.cpp
   StereoCalibration.cpp
   PinholeModelLoader.cpp
   StereoModelLoader.cpp
   Latch.cpp
   Rectifier.cpp
   DepthRegister.cpp
   RescaleRegisteredDepth.cpp
   CameraModelToCv.cpp
   PointsAccumulator.cpp
   CameraFromOpenNI.cpp
)

link_ecto(ecto_image_pipeline
          ecto_image_pipeline
          ${ecto_opencv_LIBRARIES}
)

set_target_properties(ecto_image_pipeline_ectomodule PROPERTIES
                      OUTPUT_NAME base
                      PREFIX ""
)

add_subdirectory(conversion)
add_subdirectory(g2o)
