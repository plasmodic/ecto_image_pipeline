find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()

ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

ectomodule(image_pipeline
   module.cpp
   StereoCalibration.cpp
   PinholeModelLoader.cpp
   StereoModelLoader.cpp
   Latch.cpp
   Rectifier.cpp
   DepthRegister.cpp
   CameraModelToCv.cpp
   PointsAccumulator.cpp
)

link_ecto(image_pipeline
  image_pipeline
)

install_ecto_module(image_pipeline)
