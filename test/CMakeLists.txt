find_package(GTest)

if(NOT GTEST_FOUND)
  message(STATUS "Disabling testing since gtest was not found.")
  return()
endif()

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(image_pipeline-test
  main.cpp
  pinhole.cpp
  test_pose_graph.cpp
  )

target_link_libraries(image_pipeline-test
                      ecto_image_pipeline
                      ${OpenCV_LIBS}
                      ${GTEST_LIBRARIES}
)

add_test(image_pipeline-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/image_pipeline-test)


macro(pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")

  add_test(image_pipeline_${pyfile}
    ${CMAKE_CURRENT_SOURCE_DIR}/run_test.py ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${ARGN}
    )
endmacro()

option(DESKTOP_TEST "Desktop tests will attempt to run tests that may require graphics or devices." OFF)
if(DESKTOP_TEST)
    pytest(test_source --niter 100)
endif()

if(ROS_TEST)
  pytest(test_common_io_ros_source_kinect)
  pytest(test_common_io_ros_source_bag_reader)
endif()
