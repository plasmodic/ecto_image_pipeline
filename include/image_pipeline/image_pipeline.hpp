#pragma once
#include <image_pipeline/abi.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/eigen.hpp>

#include <string>
#include <iostream>
namespace image_pipeline
{
  struct Pose //For now this is just bogus.. to test the libs.
  {
    explicit Pose(const std::string& frame_id = "",
                  const Eigen::Affine3f transform = Eigen::Affine3f::Identity()
                  );

    std::string frame_id;
    Eigen::Affine3f transform;

    friend std::ostream&
    operator<<(std::ostream& out, const image_pipeline::Pose& p);
    void toRT(cv::Mat_<float>& R, cv::Mat_<float>& T);
  };
  
}


