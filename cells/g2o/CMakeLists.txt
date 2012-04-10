find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(ecto_image_pipeline REQUIRED)

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
          cholmod
          ${ecto_image_pipeline_LIBRARIES}
          ${g2o_LIBS}
)

set_target_properties(ecto_image_pipeline_g2o_ectomodule PROPERTIES
                      OUTPUT_NAME g2o
                      PREFIX ""
)

install_ecto_module(ecto_image_pipeline_g2o)
