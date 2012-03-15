#ifndef IMAGE_PIPELINE_STEREO_CAMERA_MODEL_H
#define IMAGE_PIPELINE_STEREO_CAMERA_MODEL_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ecto_image_pipeline/image_pipeline.hpp>
#include <ecto_image_pipeline/pinhole_camera_model.h>

#include <Eigen/StdVector>

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#include <boost/make_shared.hpp>
#include <stdexcept>

#define ROS_DEPRECATED

namespace image_pipeline {

/**
 * \brief Parameter definitions for stereo cameras.
 * Holds pinhole models for the two cameras, plus the transform between them
 */
class StereoCameraModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment
    
  StereoCameraModel();

  /**
   * \brief Set up the rectification parameters
   *
   * Sets the various parameters for rectification and image size
   */
  void setParams(const PinholeCameraModel &lcam,const PinholeCameraModel &rcam, const Pose &P);

  /**
   * \brief Register a raw depth image: rectify and transform
   * 
   * Registers a depth image to this image
   * By convention, the right image is registered to the left image
   *
   * \param raw Input depth image, assumed to be rectified
   * \param registered Output registered depth image
   * \param metric Scale of the depth values, e.g., 0.001 is depth in mm
   * \param cx_offset,cy_offset Optional offset of depth image from upper left corner
   */
  void registerDepthImage(const cv::Mat& raw,
                          cv::Mat& registered,
                          const double metric,
                          int cx_offset = 0, int cy_offset = 0) const;

  /**
   * \brief Read a calibration file in YAML format
   *
   * \param filename Name of the file to read
   */
  void readCalibration(std::string filename);

  /**
   * \brief Write a calibration file in YAML format
   *
   * \param filename Name of the file to read
   */
  void writeCalibration(std::string filename) const;

  Pose pose() const{return P_;}
  PinholeCameraModel leftCamera() const{return lcam_;}
  PinholeCameraModel rightCamera() const{return rcam_;}

protected:
  Pose P_;                           // Transform from RW points to cam frame, used for registration
  PinholeCameraModel lcam_, rcam_; // Left and right stereo cameras
  cv::Mat rtemp_;                    // Temporary image for computation
};


} //namespace image_geometry

#endif
