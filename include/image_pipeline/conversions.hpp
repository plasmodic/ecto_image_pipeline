#pragma once
#include <image_pipeline/abi.hpp>
#include <image_pipeline/pose.hpp>
#include <opencv2/core/eigen.hpp>

namespace image_pipeline
{
  /**
   * Convert a Pose to OpenCV types.
   * @param pose
   * @param R 3x3 double rotation matrix
   * @param T 3x1 double translation vector
   */
  void poseToCv(const Pose& pose, cv::Mat& R, cv::Mat& T);
}
