find_package(PCL QUIET)

if(NOT PCL_FOUND)
message(WARNING "PCL could not be found, so not building conversion cells.")
return()
endif()
include_directories(${PCL_INCLUDE_DIRS})
ectomodule(image_pipeline_conversion
  module.cpp
  MatToPointCloud.cpp
  PointCloudToDepthMat.cpp
  PointCloudToMat.cpp
  )

link_ecto(image_pipeline_conversion
  ${OpenCV_LIBS}
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  )

install_ecto_module(image_pipeline_conversion)
