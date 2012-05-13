#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <string>

#include "abi.hpp"

namespace image_pipeline
{
  /**
   * Simple Pose object, with a frame id, and a transform, that is assumed to
   * be the concatenation of a rotation and translation, R|T.  This transform
   * operates in the frame specified by the frame id.
   */
  struct Pose
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    explicit
    Pose(const std::string& frame_id = "", const Eigen::Affine3d &transform = Eigen::Affine3d::Identity());

    explicit
    Pose(const std::string& frame_id, const Eigen::Matrix3d& R, const Eigen::Vector3d& T);

    std::string frame_id;
    Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> transform;

    friend std::ostream&
    operator<<(std::ostream& out, const image_pipeline::Pose& p);

    /**
     * Fill the R and T from the transform object.
     * @param R
     * @param T
     */
    void
    toRT(Eigen::Matrix3d& R, Eigen::Vector3d& T) const;

    /**
     * Reset the transform from the given R and T.
     * @param R
     * @param T
     */
    void
    fromRT(const Eigen::Matrix3d& R, const Eigen::Vector3d& T);
  };

}

