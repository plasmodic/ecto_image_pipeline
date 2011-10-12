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
  struct StereoCalibration
  {
    typedef std::vector<cv::Point3f> object_pts_t;
    typedef std::vector<object_pts_t> object_pts_v_t;
    typedef std::vector<cv::Point2f> observation_pts_t;
    typedef std::vector<observation_pts_t> observation_pts_v_t;

    static void
    declare_params(tendrils& params)
    {
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare(&StereoCalibration::image_,"image","An image to base the size of of.").required(true);
      in.declare(&StereoCalibration::object_pts_,"points_object", "The ideal object points.").required(true);
      in.declare(&StereoCalibration::pts_left_,"points_left", "The observed 2d points in the left camera.").required(true);
      in.declare(&StereoCalibration::pts_right_,"points_right", "The observed 2d points in the right camera.").required(true);
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      PinholeCameraModel left, right;
      all_object_pts.push_back(*object_pts_);
      all_left_pts.push_back(*pts_left_);
      all_right_pts.push_back(*pts_right_);
      cv::Mat image = *image_;
      image_pipeline::calibrate_stereo(all_left_pts,all_right_pts,all_object_pts,image.size(),left, right);
      return ecto::OK;
    }
    ecto::spore<object_pts_t > object_pts_;
    ecto::spore<observation_pts_t> pts_left_, pts_right_;
    object_pts_v_t all_object_pts;
    observation_pts_v_t all_left_pts, all_right_pts;
    ecto::spore<cv::Mat> image_;
  };
}
ECTO_CELL(image_pipeline, image_pipeline::StereoCalibration, "StereoCalibration",
          "Accumulates observed points and ideal 3d points, and runs "
"opencv calibration routines after some number of "
"satisfactorily unique observations.");
