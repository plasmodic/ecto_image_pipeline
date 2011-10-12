add_library(image_pipeline SHARED
  abi.cpp
  image_pipeline.cpp
  pinhole_camera_model.cpp
  stereo_camera_model.cpp
  calibration.cpp
  )

target_link_libraries(image_pipeline
  ${OpenCV_LIBS}
  )

  set_target_properties(image_pipeline PROPERTIES
    VERSION ${image_pipeline_VERSION}
    SOVERSION ${image_pipeline_SOVERSION}
    )

#install the main shared lib
install(TARGETS image_pipeline
  DESTINATION lib
  COMPONENT main
  )

