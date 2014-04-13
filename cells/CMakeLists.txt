ectomodule(base DESTINATION ecto_image_pipeline
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

link_ecto(base ecto_image_pipeline
               ${catkin_LIBRARIES}
)

add_subdirectory(conversion)
