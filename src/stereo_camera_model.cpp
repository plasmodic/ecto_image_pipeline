#include <ecto_image_pipeline/stereo_camera_model.h>
#include <boost/make_shared.hpp>
#include <exception>
#include <iostream>

#include <opencv2/core/eigen.hpp>

namespace image_pipeline
{

  StereoCameraModel::StereoCameraModel()
  {
  }

// initialize the camera model
  void
  StereoCameraModel::setParams(const PinholeCameraModel &lcam, const PinholeCameraModel &rcam, const Pose &P)
  {
    P_ = P;
    lcam_ = lcam;
    rcam_ = rcam;
  }

// lcam is rgb cam
// rcam is depth cam

  void
  StereoCameraModel::registerDepthImage(const cv::Mat& raw, cv::Mat& registered, double metric, 
                                        int cx_offset, int cy_offset) const
  {
    if (raw.type() != CV_16UC1)
      throw std::runtime_error("Bad image type. expecting 16UC1");

    // create reprojection matrix Q
    Eigen::Matrix4d Q;
    Q << 1, 0, 0, -rcam_.cx(), 0, 1, 0, -rcam_.cy(), 0, 0, 0, rcam_.fx(), 0, 0, rcam_.fx(), 0;

    // form the extended camera matrix [K 0; 0 1]
    Eigen::Matrix4d K;
    K.setIdentity();
    K.block(0, 0, 3, 3) = lcam_.Kp_; // camera matrix

    // form the projection matrix via concatenation
    Eigen::Matrix4d P;
    Eigen::Matrix4f Pf;

    P = K * P_.transform.matrix().inverse() * Q; //left * Transform * right_reprojection
    Pf = P.cast<float>();

    // for each element of the depth image, transform to output image
    // set output matrix size and zero it
    // we have to unfortunately assume the type is CV_16UC to get access
    registered.create(raw.rows, raw.cols, raw.type());
    registered.setTo(0);
    float im = 1.0 / metric;
    int rows = registered.rows;
    int cols = registered.cols;
    for (int i = 0; i < rows; i++)
      for (int j = 0; j < cols; j++)
      {
        int d = raw.at<uint16_t>(i, j);
        if (d > 0)
        {
          Eigen::Vector4f v;
          v << (float) j+cx_offset, (float) i+cy_offset, im / (float) d, 1.0f;
          Eigen::Vector4f vp = Pf * v; // transformed
          if (vp(3) > 0.0)
          {
            uint16_t z = (uint16_t) (im * vp(2) / vp(3));
            int u = (int) (vp(0) / vp(2) + 0.5);
            int v = (int) (vp(1) / vp(2) + 0.5);
            if (u > 0 && u < cols && v > 0 && v < rows)
            {
              int zz = registered.at<uint16_t>(v, u);
              if (zz == 0 || z < zz) // check Z buffer
                registered.at<uint16_t>(v, u) = z;
            }

          }
        }
      }
  }

  void
  StereoCameraModel::writeCalibration(std::string filename) const
  {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    CV_Assert(fs.isOpened());
    cv::Mat P;
    cv::eigen2cv(P_.transform.matrix(), P);

    cvWriteComment(*fs, "Stereo", 0);

    if (!P.empty())
      fs << "stereo_pose_offset" << P;
  }

  void
  StereoCameraModel::readCalibration(std::string filename)
  {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    CV_Assert(fs.isOpened());
    cv::Mat P;
    cv::read(fs["stereo_pose_offset"], P, cv::Mat());

    CV_Assert(P.empty() == false);
    Eigen::Matrix4d Px;
    cv2eigen(P, Px);
    P_.transform.matrix() = Px;
  }

}
