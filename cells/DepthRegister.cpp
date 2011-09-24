#include <ecto/ecto.hpp>
#include <image_pipeline/pinhole_camera_model.h>

using ecto::tendrils;

namespace image_pipeline
{
  struct DepthRegister
  {
    static void
    declare_params(tendrils& params)
    {
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<PinholeCameraModel>("rgb_camera", "Camera model.").required(true);
      in.declare<PinholeCameraModel>("depth_camera", "Camera model.").required(true);
      in.declare<cv::Mat>("image", "The input image.").required(true);
      out.declare<cv::Mat>("image", "The registered image.");
    }
    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      rgb_camera = in["rgb_camera"]; 
      depth_camera = in["depth_camera"]; 
      image_in = in["image"];
      image_out = out["image"];
      //      cv::Mat output;   // do this for non-copied output
      //      *image_out = output;
    }
    int process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      cv::Mat output;
      rgb_camera->registerDepthImage(*image_in,*depth_camera,output,0.001);
      *image_out = output;
      return ecto::OK;
    }
    ecto::spore<PinholeCameraModel> rgb_camera, depth_camera;
    ecto::spore<cv::Mat> image_in, image_out;

  };
}

ECTO_CELL(image_pipeline, image_pipeline::DepthRegister, "DepthRegister",
          "Given a depth image and rgb image model, register the depth image.");
