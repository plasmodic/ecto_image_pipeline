#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>

using ecto::tendrils;
namespace image_pipeline
{
  struct RescaledRegisteredDepth
  {
    static void
    declare_params(tendrils& params)
    {
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("depth", "The depth image to rescale.").required(true);
      in.declare<cv::Mat>("image", "The rgb image.").required(true);
      out.declare<cv::Mat>("depth", "The rescaled depth image.");
    }
    void
    configure(const tendrils& p, const tendrils& inputs, const tendrils& outputs)
    {
      depth_in = inputs["depth"];
      image_in = inputs["image"];
      depth_out = outputs["depth"];
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Size dsize = depth_in->size(), isize = image_in->size();
      cv::Mat depth;
      cv::Mat valid_mask;
      rescaleDepth(*depth_in, CV_32F, depth);

      if (dsize == isize)
      {
        *depth_out = depth;
        return ecto::OK;
      }

      float factor = float(isize.width) / dsize.width; //scaling factor.
      cv::Mat output(isize, depth.type(), std::numeric_limits<float>::quiet_NaN()); //output is same size as image.
      //resize into the subregion of the correct aspect ratio
      cv::Mat subregion(output.rowRange(0, dsize.height * factor));
      //use nearest neighbor to prevent discontinuities causing bogus depth.
      cv::resize(depth, subregion, subregion.size(), CV_INTER_NN);
      *depth_out = output;
      return ecto::OK;
    }
    ecto::spore<cv::Mat> image_in, depth_in, depth_out;
  };
}
ECTO_CELL(base, image_pipeline::RescaledRegisteredDepth, "RescaledRegisteredDepth",
          "Rescale the openni depth image to be the same size as the image if necessary.");
