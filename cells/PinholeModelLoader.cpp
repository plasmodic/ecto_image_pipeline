#include <ecto/ecto.hpp>
#include <ecto_image_pipeline/pinhole_camera_model.h>

#include <iostream>

using namespace Eigen;

using ecto::tendrils;

namespace image_pipeline
{
  struct PinholeModelLoader
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("filename", "The camera calibration file.", "camera.yml");
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      out.declare<PinholeCameraModel>("model", "The camera model loaded.");
    }
    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      camera = out["model"]; // this is like a smart pointer to the output.
      PinholeCameraModel cam;

      // some dummy params
      Eigen::Matrix3d K = Matrix3d::Zero();
      K(0,0) = 400; K(1,1) = 400; K(0,2) = 320; K(1,2) = 240; K(2,2) = 1;
      Eigen::Matrix3d R = Matrix3d::Identity();
      Eigen::VectorXd D(5,1);
      D << 0.1, 0, 0, 0, 0;
      cv::Size s(640,480);
      cam.setParams(s,K,D,R,K);

      std::string filename;
      params["filename"] >> filename;
      cam.readCalibration(filename);
      cam.writeCalibration("xx.yml");
      *camera = cam;

      std::cout << "Config of Pinhole Model Loader" << std::endl;
      //TODO load the camera from file.
      //load(*camera, filename);
    }
    ecto::spore<PinholeCameraModel> camera;
  };
}

ECTO_CELL(base, image_pipeline::PinholeModelLoader, "PinholeModelLoader",
          "This reads a monocular camera calibration file and puts the results on the output.");
