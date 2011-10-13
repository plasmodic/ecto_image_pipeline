#include <ecto/ecto.hpp>
#include <image_pipeline/stereo_camera_model.h>

#include <iostream>

using namespace Eigen;

using ecto::tendrils;

namespace image_pipeline
{
  struct StereoModelLoader
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("left_fname", "The left camera calibration file.", "left.yml");
      params.declare<std::string>("right_fname", "The right camera calibration file.", "right.yml");
      params.declare<std::string>("stereo_fname", "The stereo camera calibration file.", "stereo.yml");
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      out.declare<StereoCameraModel>("stereo_model", "The stereo camera model loaded.");
      out.declare<PinholeCameraModel>("left_model", "The left camera model loaded.");
      out.declare<PinholeCameraModel>("right_model", "The right camera model loaded.");
    }
    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      scam = out["stereo_model"]; // this is like a smart pointer to the output.
      lcam = out["left_model"]; // this is like a smart pointer to the output.
      rcam = out["right_model"]; // this is like a smart pointer to the output.
      PinholeCameraModel cam;
      StereoCameraModel stereo;

      // some dummy params
      Eigen::Matrix3d K = Matrix3d::Zero();
      K(0,0) = 400; K(1,1) = 400; K(0,2) = 320; K(1,2) = 240; K(2,2) = 1;
      Eigen::Matrix3d R = Matrix3d::Identity();
      Eigen::VectorXd D(5,1);
      D << 0.1, 0, 0, 0, 0;
      cv::Size s(640,480);
      cam.setParams(s,K,D,R,K);
      *lcam = cam;
      *rcam = cam;
      *scam = stereo;

      std::string filename;
#if 1
      params["left_fname"] >> filename;
      cam.readCalibration(filename);
      cam.writeCalibration("xx.yml");
      *lcam = cam;

      params["right_fname"] >> filename;
      cam.readCalibration(filename);
      cam.writeCalibration("xx.yml");
      *rcam = cam;

      params["stereo_fname"] >> filename;
      stereo.readCalibration(filename);
      stereo.writeCalibration("ss.yml");
      *scam = stereo;
#endif

      std::cout << "Config of Stereo Model Loader" << std::endl;
    }
    ecto::spore<PinholeCameraModel> lcam, rcam;
    ecto::spore<StereoCameraModel> scam;
  };
}

ECTO_CELL(image_pipeline, image_pipeline::StereoModelLoader, "StereoModelLoader",
          "This reads a stereo camera calibration file and two monocular cal files, and puts the results on the outputs.");
