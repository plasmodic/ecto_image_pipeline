#include <ecto_image_pipeline/pose.hpp>

namespace image_pipeline
{

  Pose::Pose(const std::string& frame_id, const Eigen::Affine3d& transform)
    :
    frame_id(frame_id),
    transform(transform)
  {
  }
  Pose::Pose(const std::string& frame_id, const Eigen::Matrix3d& R, const Eigen::Vector3d& T)
      :
        frame_id(frame_id)
  {
    fromRT(R, T);
  }
  std::ostream&
  operator<<(std::ostream& out, const image_pipeline::Pose& p)
  {
    return out << p.frame_id << ":\n" << p.transform.matrix();
  }

  void
  Pose::toRT(Eigen::Matrix3d& R, Eigen::Vector3d& T) const
  {
    R = transform.rotation();
    T = transform.translation();
  }

  void
  Pose::fromRT(const Eigen::Matrix3d& R, const Eigen::Vector3d& T)
  {
    transform.setIdentity();
    transform.rotate(R);
    transform.translate(T);
  }
}

