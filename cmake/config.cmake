set(image_pipeline_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
set(image_pipeline_LIBRARIES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libimage_pipeline.so)
configure_file(cmake/image_pipelineConfig.cmake.in
  ${CMAKE_BINARY_DIR}/image_pipelineConfig.cmake
  @ONLY
)
configure_file(cmake/image_pipelineConfig-version.cmake.in
  ${CMAKE_BINARY_DIR}/image_pipelineConfig-version.cmake
  @ONLY
)

#this is a bit simple.
set(image_pipeline_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/${prefix})
set(image_pipeline_LIBRARIES ${CMAKE_INSTALL_PREFIX}/lib/libimage_pipeline.so)
configure_file(cmake/image_pipelineConfig.cmake.in
  ${PROJECT_BINARY_DIR}/share/image_pipelineConfig.cmake
  @ONLY
)
configure_file(cmake/image_pipelineConfig-version.cmake.in
  ${CMAKE_BINARY_DIR}/share/image_pipelineConfig-version.cmake
  @ONLY
)
