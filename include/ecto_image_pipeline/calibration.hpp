#pragma once

#include "abi.hpp"
#include "stereo_camera_model.h"

namespace image_pipeline
{
  typedef std::vector<cv::Point3f> object_pts_t;
  typedef std::vector<object_pts_t> object_pts_v_t;
  typedef std::vector<cv::Point2f> observation_pts_t;
  typedef std::vector<observation_pts_t> observation_pts_v_t;

  void
  calibrate_stereo(const observation_pts_v_t& left_points, const observation_pts_v_t& right_points,
                   const object_pts_v_t& object_points, const cv::Size& image_size, PinholeCameraModel& left, PinholeCameraModel& right);

}

