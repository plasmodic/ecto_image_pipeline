install(DIRECTORY ${image_pipeline_SOURCE_DIR}/include/
  DESTINATION include
  COMPONENT main
  )

#install the unix_install
install(DIRECTORY ${image_pipeline_BINARY_DIR}/share/
        DESTINATION share/image_pipeline
        COMPONENT main
)

