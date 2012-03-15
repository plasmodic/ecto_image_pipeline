install(DIRECTORY ${ecto_image_pipeline_SOURCE_DIR}/include/
  DESTINATION include
  COMPONENT main
  )

#install the unix_install
install(DIRECTORY ${ecto_image_pipeline_BINARY_DIR}/share/
        DESTINATION share/ecto_image_pipeline
        COMPONENT main
)

