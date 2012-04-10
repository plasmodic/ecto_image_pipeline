find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(ecto_image_pipeline REQUIRED)

set_ecto_install_package_name()

ectomodule(ecto_image_pipeline_g2o
           module.cpp
           Sba.cpp
           SbaDisparity.cpp
)

find_package(g2o)
include_directories(SYSTEM
                    /usr/include/suitesparse
                    ${g2o_INCLUDE_DIRS}
)
link_ecto(ecto_image_pipeline_g2o
          ${ecto_image_pipeline_LIBRARIES}
          ${g2o_LIBS}
)

install_ecto_module(ecto_image_pipeline_g2o)
