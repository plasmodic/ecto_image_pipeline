#include <Eigen/Core>

#include <ecto_image_pipeline/conversions.hpp>

namespace image_pipeline
{
  void
  poseToCv(const Pose& pose, cv::Mat& R, cv::Mat& T)
  {
    Eigen::Matrix3d eR;
    Eigen::Vector3d eT;
    pose.toRT(eR, eT);
    cv::eigen2cv(eR, R);
    cv::eigen2cv(eT, T);
  }
}
