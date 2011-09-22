set(prefix image_pipeline-${image_pipeline_VERSION})
install(DIRECTORY ${image_pipeline_SOURCE_DIR}/include/
  DESTINATION include/${prefix}
  COMPONENT main
  )

#install the unix_install
install(DIRECTORY ${image_pipeline_BINARY_DIR}/share/
  DESTINATION share/${prefix}
  COMPONENT main
  )

