catkin_add_gtest(image_pipeline-test
  main.cpp
  pinhole.cpp
  test_pose_graph.cpp
  )

target_link_libraries(image_pipeline-test
                      ecto_image_pipeline
                      ${catkin_LIBRARIES}
)

catkin_add_nosetests(test_common_io_ros_source_kinect.py)
catkin_add_nosetests(test_common_io_ros_source_bag_reader.py)
