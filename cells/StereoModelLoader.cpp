#include <ecto/ecto.hpp>
#include <ecto_image_pipeline/stereo_camera_model.h>

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
      PinholeCameraModel left_cam, right_cam;
      StereoCameraModel stereo;

      std::string filename;
      params["left_fname"] >> filename;
      lcam->readCalibration(filename);

      params["right_fname"] >> filename;
      rcam->readCalibration(filename);

      params["stereo_fname"] >> filename;
      scam->readCalibration(filename);
      scam->setParams(*lcam, *rcam, scam->pose());
    }
    ecto::spore<PinholeCameraModel> lcam, rcam;
    ecto::spore<StereoCameraModel> scam;
  };
}

ECTO_CELL(base, image_pipeline::StereoModelLoader, "StereoModelLoader",
    "This reads a stereo camera calibration file and two monocular cal files, and puts the results on the outputs.");
