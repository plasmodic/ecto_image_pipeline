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

set_target_properties(ecto_image_pipeline_ectomodule PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${python_LIBRARY_OUTPUT_DIRECTORY}
                      OUTPUT_NAME base
                      PREFIX ""
)

 
install(TARGETS ecto_image_pipeline_ectomodule
        DESTINATION ${PYTHON_PACKAGES_PATH}/ecto_image_pipeline
)

add_subdirectory(conversion)
