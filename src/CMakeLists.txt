add_library(ecto_image_pipeline SHARED
            abi.cpp
            calibration.cpp
            conversions.cpp
            pinhole_camera_model.cpp
            pose.cpp
            pose_graph.cpp
            stereo_camera_model.cpp
)

target_link_libraries(ecto_image_pipeline
                      ${OpenCV_LIBS}
                      ${OpenCV_LIBRARIES}
)

#install the main shared lib
install(TARGETS ecto_image_pipeline
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        COMPONENT main
)
