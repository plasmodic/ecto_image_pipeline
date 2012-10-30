find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()

return()

find_package(g2o QUIET)
if(NOT g2o_FOUND)
  find_package(libg2o QUIET)
  if(NOT libg2o_FOUND)
    return()
  endif()
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

ectomodule(ecto_image_pipeline_g2o DESTINATION ecto_image_pipeline
                                   INSTALL
           module.cpp
           Sba.cpp
           SbaDisparity.cpp
)

include_directories(SYSTEM
                    /usr/include/suitesparse
                    ${g2o_INCLUDE_DIRS}
)
link_ecto(ecto_image_pipeline_g2o
          cholmod
          ${g2o_LIBS}
)

set_target_properties(ecto_image_pipeline_g2o_ectomodule PROPERTIES
                      OUTPUT_NAME g2o
                      PREFIX ""
)
