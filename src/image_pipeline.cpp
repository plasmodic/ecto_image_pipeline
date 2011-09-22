#include <image_pipeline/image_pipeline.hpp>

namespace  image_pipeline {

  Pose::Pose(const std::string& frame_id, const Eigen::Affine3f transform)
    :
      frame_id(frame_id),
      transform(transform)
  {
  }

  std::ostream&
  operator<<(std::ostream& out, const image_pipeline::Pose& p)
  {
    return out << p.frame_id << ":\n" << p.transform.matrix();
  }

}

