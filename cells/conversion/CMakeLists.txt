find_package(PCL QUIET)

if(NOT PCL_FOUND)
message(WARNING "PCL could not be found, so not building conversion cells.")
return()
endif()

include_directories(SYSTEM
                    ${PCL_INCLUDE_DIRS}
)

ectomodule(conversion DESTINATION ecto_image_pipeline
                      INSTALL
  module.cpp
  MatToPointCloud.cpp
  PointCloudToDepthMat.cpp
  PointCloudToMat.cpp
  )

link_ecto(conversion
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  )
