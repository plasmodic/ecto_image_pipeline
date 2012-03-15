#include <ecto/ecto.hpp>
#include <ecto_image_pipeline/stereo_camera_model.h>

#include <iostream>

using namespace Eigen;

using ecto::tendrils;

namespace image_pipeline
{
  struct CameraFromOpenNI
  {
    typedef CameraFromOpenNI C;
    static void
    declare_io(const tendrils& params, tendrils& i, tendrils& o)
    {
      //inputs coming off of the kinect.
      i.declare(&C::depth_, "depth", "The depth stream.").required(true);
      i.declare(&C::image_, "image", "The image stream.").required(true);
      i.declare(&C::focal_length_image_, "focal_length_image", "The focal length of the image stream.").required(true);
      i.declare(&C::focal_length_depth_, "focal_length_depth", "The focal length of the depth stream.").required(true);
      i.declare(&C::baseline_, "baseline", "The base line of the openni camera.").required(true);

      o.declare(&C::cam, "camera", "A pinhole camera model to convert.");
    }

    cv::Mat
    createK(float focal_length, const cv::Mat& image) const
    {
      cv::Mat K_;
      K_.create(3, 3, CV_64FC1);
      cv::Mat_<double> K = K_;
      K = 0;
      K(0, 0) = K(1, 1) = focal_length;
      K(0, 2) = image.size().width / 2 + 0.5;
      K(1, 2) = image.size().height / 2 + 0.5;
      K(2, 2) = 1;
      return K_;
    }
    int
    process(const tendrils& in, const tendrils& out)
    {

      cv::Mat K_left = createK(*focal_length_image_, *image_);
      left_.setParams(image_->size(), K_left, cv::Mat(), cv::Mat_<double>::eye(3, 3), K_left, 0, 0);

      //cv::Mat K_right = createK(*focal_length_depth_, *depth_);
      //right_.setParams(image_->size(), K_right, cv::Mat(), cv::Mat_<double>::eye(3, 3), K_left, 0, 0);
      //cam->setParams(left_, right_, Pose("/rgb_camera", Matrix3d::Identity(), Vector3d(*baseline_, 0, 0)));
      *cam = left_;
      return ecto::OK;
    }
    PinholeCameraModel left_, right_;
    StereoCameraModel stereo_camera_;
    ecto::spore<PinholeCameraModel> cam;
    ecto::spore<cv::Mat> depth_, image_;
    ecto::spore<float> focal_length_image_, focal_length_depth_, baseline_;
  };
}

ECTO_CELL(base, image_pipeline::CameraFromOpenNI, "CameraFromOpenNI",
          "Creates a camera model from an OpenNI source.");
