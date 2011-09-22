#include <gtest/gtest.h>
#include <image_pipeline/image_pipeline.hpp>

TEST(pinhole, Test1)
{
  image_pipeline::Pose p("/base");
  std::cout << p << std::endl;
  EXPECT_TRUE(p.transform.isApprox(Eigen::Affine3f::Identity()));
}
