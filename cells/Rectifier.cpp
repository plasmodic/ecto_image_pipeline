#include <ecto/ecto.hpp>
#include <ecto_image_pipeline/pinhole_camera_model.h>
#include <ecto_image_pipeline/enums.hpp>

using ecto::tendrils;

namespace image_pipeline
{
  struct Rectifier
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<InterpolationMode>("interpolation_mode", "Interpolation method for rectification", CV_INTER_LINEAR);
      params.declare<int>("cx_offset", "Center offset X of input image", 0);
      params.declare<int>("cy_offset", "Center offset Y of input image", 0);
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
      camera = in["camera"];    // original pinhole model
      params["cx_offset"] >> cx_offset;
      params["cy_offset"] >> cy_offset;
      params["interpolation_mode"] >> mode;
      image_in = in["image"];
      image_out = out["image"];
    }
    int process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      // this is ugly; should really compare stored rect params to incoming ones each time
      if (camoffset != *camera)
        {
          camoffset = *camera;      // modified model
          camoffset.initCache();
          //          std::cout << "Offsets: " << cx_offset << " " << cy_offset << std::endl;
          //          std::cout << camoffset.getK() << std::endl;
          //          std::cout << camera->getK() << std::endl;
          if (cx_offset != 0 || cy_offset != 0)
            camoffset.setCenterOffset(cx_offset,cy_offset);
        }

      cv::Mat output;
      camoffset.rectifyImage(*image_in,output,mode);
      *image_out = output;
      return ecto::OK;
    }
    ecto::spore<PinholeCameraModel> camera;
    ecto::spore<cv::Mat> image_in, image_out;
    InterpolationMode mode;
    PinholeCameraModel camoffset;
    int cx_offset, cy_offset;
  };
}

ECTO_CELL(base, image_pipeline::Rectifier, "Rectifier",
          "Given a PinholeCameraModel, rectify the input image.");
