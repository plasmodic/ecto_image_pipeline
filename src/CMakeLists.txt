
add_library(image_pipeline
  abi.cpp
  )

target_link_libraries(image_pipeline
  ${OpenCV_LIBS}
  )

