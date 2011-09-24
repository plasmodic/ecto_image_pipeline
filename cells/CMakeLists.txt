find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()

ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

ectomodule(image_pipeline
   module.cpp
   CalibrationLoader.cpp
   Rectifier.cpp
   RectifierNC.cpp
   DepthRegister.cpp
)

link_ecto(image_pipeline
  image_pipeline
)

