find_pcl_package()

if(NOT PCL_FOUND)
message(WARNING "PCL could not be found, so not building conversion cells.")
return()
endif()

include_directories(SYSTEM
                    ${PCL_INCLUDE_DIRS}
)

ectomodule(ecto_image_pipeline_conversion
  module.cpp
  MatToPointCloud.cpp
  PointCloudToDepthMat.cpp
  PointCloudToMat.cpp
  )

link_ecto(ecto_image_pipeline_conversion
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  )

set_target_properties(ecto_image_pipeline_conversion_ectomodule PROPERTIES
                      OUTPUT_NAME conversion
                      PREFIX ""
)

install_ecto_module(ecto_image_pipeline_conversion)
