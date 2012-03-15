#include <ecto/ecto.hpp>
#include <ecto_image_pipeline/stereo_camera_model.h>

#include <iostream>

using namespace Eigen;

using ecto::tendrils;

namespace image_pipeline
{
  struct CameraModelToCv
  {
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare(&CameraModelToCv::cam, "camera", "A pinhole camera model to convert.");
      out.declare(&CameraModelToCv::K, "K", "The camera matrix.");
      out.declare(&CameraModelToCv::Kp, "Kp", "The output camera matrix.");
      out.declare(&CameraModelToCv::D, "D", "The distortion coefficients.");
      out.declare(&CameraModelToCv::R, "R", "The rotation matrix.");
      out.declare(&CameraModelToCv::image_size, "image_size", "The size of the image.");
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cam->toCv(*image_size, *K, *D, *R, *Kp);
      return ecto::OK;
    }
    ecto::spore<PinholeCameraModel> cam;
    ecto::spore<cv::Mat> K, Kp, R, D;
    ecto::spore<cv::Size> image_size;
  };
}

ECTO_CELL(base, image_pipeline::CameraModelToCv, "CameraModelToCv",
          "Expands an image_pipeline camera to opencv types.");
