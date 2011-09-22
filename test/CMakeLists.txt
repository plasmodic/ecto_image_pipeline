find_package(GTest)

if(NOT GTEST_FOUND)
  message(STATUS "Disabling testing since gtest was not found.")
  return()
endif()

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(image_pipeline-test
  main.cpp
  pinhole.cpp
  )

target_link_libraries(image_pipeline-test
  image_pipeline
  ${OpenCV_LIBS}
  ${GTEST_LIBRARIES}
  )

add_test(image_pipeline-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/image_pipeline-test)
