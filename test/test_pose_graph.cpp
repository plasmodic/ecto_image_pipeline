#include <gtest/gtest.h>
#include <image_pipeline/image_pipeline.hpp>
#include <image_pipeline/pose_graph.hpp>
#include <image_pipeline/pinhole_camera_model.h>
using namespace image_pipeline;

TEST(pose_graph, PoseGraphBasics)
{
  PoseGraph::transform //
  t_ab = Eigen::Affine3d(Eigen::Translation3d(0, 0, 1)), //
  t_bc = Eigen::Affine3d(Eigen::Translation3d(0, 0, -1)), //
  t_bd = Eigen::Affine3d(Eigen::Translation3d(0, 0, 1)), //
  t_xx;

  PoseGraph pose_graph;
  pose_graph.set("/a", "/b", t_ab);
  pose_graph.set("/b", "/c", t_bc);
  pose_graph.set("/b", "/d", t_bd);

  pose_graph.lookup("/a", "/b", t_xx);
  EXPECT_EQ(t_xx.translation()[2], 1.0);

  pose_graph.lookup("/a", "/d", t_xx);
  EXPECT_EQ(t_xx.translation()[2], 2.0);

  pose_graph.lookup("/d", "/c", t_xx);
  EXPECT_EQ(t_xx.translation()[2], -2.0);

  pose_graph.lookup("/c", "/d", t_xx);
  EXPECT_EQ(t_xx.translation()[2], 2.0);

  EXPECT_FALSE(pose_graph.lookup("/d", "/x", t_xx));
}

TEST(pose_graph, PoseGraphFail)
{
  PoseGraph::transform //
  t_ab = Eigen::Affine3d(Eigen::Translation3d(0, 0, 1)), //
  t_bc = Eigen::Affine3d(Eigen::Translation3d(0, 0, -1)), //
  t_bd = Eigen::Affine3d(Eigen::Translation3d(0, 0, 1)), //
  t_xx;

  PoseGraph pose_graph;
  pose_graph.set("/a", "/x", t_ab);
  pose_graph.set("/b", "/c", t_bc);
  pose_graph.set("/b", "/d", t_bd);

  EXPECT_FALSE(pose_graph.lookup("/a", "/b", t_xx));
  EXPECT_FALSE(pose_graph.lookup("/f", "/b", t_xx));
  EXPECT_FALSE(pose_graph.lookup("/h", "/g", t_xx));
}

TEST(pose_graph, PoseGraphEmpty)
{
  PoseGraph::transform t_xx;
  PoseGraph pose_graph_empty;
  EXPECT_FALSE(pose_graph_empty.lookup("/a", "/b", t_xx));
}

TEST(pose_graph, PoseGraphUpdate)
{
  PoseGraph::transform //
  t_ab = Eigen::Affine3d(Eigen::Translation3d(0, 0, 1)), //
  t_bc = Eigen::Affine3d(Eigen::Translation3d(0, 0, -1)), //
  t_bd = Eigen::Affine3d(Eigen::Translation3d(0, 0, 1)), //
  t_xx;

  PoseGraph pose_graph;
  pose_graph.set("/a", "/b", t_ab);
  pose_graph.set("/b", "/c", t_bc);
  pose_graph.set("/b", "/d", t_bd);
  pose_graph.lookup("/a", "/d", t_xx);
  EXPECT_EQ(t_xx.translation()[2], 2.0);
  EXPECT_EQ(pose_graph("/c","/d").translation()[2], 2.0);
  t_bd *= Eigen::Translation3d(0, 0, 3);
  pose_graph.set("/b", "/d", t_bd);
  EXPECT_EQ(pose_graph("/c","/d").translation()[2], 5.0);
  EXPECT_EQ(pose_graph("/a","/b").translation()[2], 1.0);

}

