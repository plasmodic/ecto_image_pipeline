#include <ecto/ecto.hpp>
#include <image_pipeline/pinhole_camera_model.h>

using ecto::tendrils;

namespace image_pipeline
{
  struct Rectifier
  {
    static void
    declare_params(tendrils& params)
    {
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<PinholeCameraModel>("camera", "Camera model.").required(true);
      in.declare<cv::Mat>("image", "The input image.").required(true);
      out.declare<cv::Mat>("image", "The rectified image.");
    }
    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      camera = in["camera"]; // this is like a smart pointer to the output.
      image_in = in["image"];
      image_out = out["image"];
    }
    int process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      cv::Mat output;
      camera->rectifyImage(*image_in,output);
      *image_out = output;
      return ecto::OK;
    }
    ecto::spore<PinholeCameraModel> camera;
    ecto::spore<cv::Mat> image_in, image_out;

  };
}

ECTO_CELL(image_pipeline, image_pipeline::Rectifier, "Rectifier",
          "Given a PinholeCameraModel, rectify the input image.");
