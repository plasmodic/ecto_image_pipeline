#install targets for all things python
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ecto_image_pipeline
  DESTINATION ${PYTHON_PACKAGES_PATH}
  COMPONENT main
  FILES_MATCHING PATTERN "*.py"
)

configure_file(__init__.py.in
               ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
               @ONLY
)
