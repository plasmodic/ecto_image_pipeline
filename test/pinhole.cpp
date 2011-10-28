#include <gtest/gtest.h>
#include <image_pipeline/image_pipeline.hpp>
#include <image_pipeline/pinhole_camera_model.h>
TEST(pinhole, Bogus1)
{
  image_pipeline::Pose p("/base");
  std::cout << p << std::endl;
  EXPECT_TRUE(p.transform.isApprox(Eigen::Affine3d::Identity()));
}

TEST(pinhole, Serialization)
{
  
}
