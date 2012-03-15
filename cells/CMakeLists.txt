find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()


ectomodule(ecto_image_pipeline
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
)

install_ecto_module(ecto_image_pipeline)

add_subdirectory(conversion)
