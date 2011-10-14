#pragma once
#include <image_pipeline/abi.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/eigen.hpp>

#include <string>
#include <iostream>
namespace image_pipeline
{
  struct Pose
  {
    explicit Pose(const std::string& frame_id = "",
                  const Eigen::Affine3d transform = Eigen::Affine3d::Identity()
                  );

    std::string frame_id;
    Eigen::Affine3d transform;

    friend std::ostream&
    operator<<(std::ostream& out, const image_pipeline::Pose& p);
    void toRT(Eigen::Matrix3d& R, Eigen::Vector3d& T);
  };
  
}


