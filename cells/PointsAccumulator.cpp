#include <ecto/ecto.hpp>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <image_pipeline/image_pipeline.hpp>
#include <image_pipeline/calibration.hpp>
using ecto::tendrils;
namespace image_pipeline
{
  template<typename PointT>
  struct PointsAccumulator
  {
    typedef std::vector<PointT> pts_t;
    typedef std::vector<pts_t> pts_v_t;

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare(&PointsAccumulator::pts_, "points", "A set of points, that will be stacked.").required(true);
      out.declare(&PointsAccumulator::stacked_, "stacked", "The total stack of points.");
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      stacked_->push_back(*pts_);
      return ecto::OK;
    }
    ecto::spore<pts_t> pts_;
    ecto::spore<pts_v_t> stacked_;
  };
}

ECTO_CELL(image_pipeline, image_pipeline::PointsAccumulator<cv::Point2f>, "Points2DAccumulator",
          "Accumulates 2D points.");

ECTO_CELL(image_pipeline, image_pipeline::PointsAccumulator<cv::Point3f>, "Points3DAccumulator",
          "Accumulates 3D points.");
