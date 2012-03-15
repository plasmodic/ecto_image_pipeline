#ifndef IMAGE_PIPELINE_PINHOLE_CAMERA_MODEL_H
#define IMAGE_PIPELINE_PINHOLE_CAMERA_MODEL_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "image_pipeline.hpp"

#include <Eigen/StdVector>

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#include <boost/make_shared.hpp>
#include <stdexcept>

#define ROS_DEPRECATED

namespace image_pipeline {

/**
 * \brief Parameter definitions for monocular pinhole cameras
 */
class PinholeCameraModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment
    
  PinholeCameraModel();

  /**
   * \brief Inequality operator
   *
   * Compares two camera models, returns true if they differ in a critical parameter 
   */
  bool operator != (const PinholeCameraModel& mCamObj);

  /**
   * \brief Clear cache
   *
   * Clears the cache and initializes to a local pointer
   */
  void initCache();

  /**
   * \brief Set up the rectification parameters
   *
   * Sets the various parameters for rectification and image size
   */
  void setParams(cv::Size &size, Eigen::Matrix3d &K, Eigen::VectorXd &D, Eigen::Matrix3d &R, Eigen::Matrix3d &Kp,
                 const double ox = 0.0, const double oy = 0.0);
  void setParams(cv::Size image_size, const cv::Mat& K, const cv::Mat& D, const cv::Mat& R, const cv::Mat&Kp,
                 const double ox = 0.0, const double oy = 0.0);
  void setCenterOffset(const double ox, const double oy);
  void toCv(cv::Size &size,cv::Mat &K,  cv::Mat &D, cv::Mat &R,
                                 cv::Mat &Kp);
  /**
   * \brief The resolution at which the camera was calibrated.
   *
   * The maximum resolution at which the camera can be used with the current
   * calibration; normally this is the same as the imager resolution.
   */
  cv::Size fullResolution() const;

  /**
   * \brief The resolution of the current rectified image.
   *
   * The size of the rectified image associated with the latest CameraInfo, as
   * reduced by binning/ROI and affected by distortion. If binning and ROI are
   * not in use, this is the same as fullResolution().
   */
  cv::Size reducedResolution() const;

  Eigen::Vector2d toFullResolution(const Eigen::Vector2d& uv_reduced) const;

  cv::Rect toFullResolution(const cv::Rect& roi_reduced) const;

  Eigen::Vector2d toReducedResolution(const Eigen::Vector2d& uv_full) const;

  cv::Rect toReducedResolution(const cv::Rect& roi_full) const;

  /**
   * \brief The current raw ROI, as used for capture by the camera driver.
   */
  cv::Rect rawRoi() const;

  /**
   * \brief The current rectified ROI, which best fits the raw ROI.
   */
  cv::Rect rectifiedRoi() const;

  /// @todo Hide or group deprecated overloads in Doxygen
  void project3dToPixel(const Eigen::Vector3d& xyz, Eigen::Vector2d& uv_rect) const ROS_DEPRECATED;
  
  /**
   * \brief Project a 3d point to rectified pixel coordinates.
   *
   * This is the inverse of projectPixelTo3dRay().
   *
   * \param xyz 3d point in the camera coordinate frame
   * \return (u,v) in rectified pixel coordinates
   */
  Eigen::Vector2d project3dToPixel(const Eigen::Vector3d& xyz) const;

  void projectPixelTo3dRay(const Eigen::Vector2d& uv_rect, Eigen::Vector3d& ray) const ROS_DEPRECATED;

  /**
   * \brief Project a rectified pixel to a 3d ray.
   *
   * Returns the unit vector in the camera coordinate frame in the direction of rectified
   * pixel (u,v) in the image plane. This is the inverse of project3dToPixel().
   *
   * In 1.4.x, the vector has z = 1.0. Previously, this function returned a unit vector.
   *
   * \param uv_rect Rectified pixel coordinates
   * \return 3d ray passing through (u,v)
   */
  Eigen::Vector3d projectPixelTo3dRay(const Eigen::Vector2d& uv_rect) const;

  /**
   * \brief Rectify a raw camera image.
   */
  void rectifyImage(const cv::Mat& raw, cv::Mat& rectified,
                    int interpolation = CV_INTER_LINEAR) const;

  /**
   * \brief Apply camera distortion to a rectified image.
   */
  void unrectifyImage(const cv::Mat& rectified, cv::Mat& raw,
                      int interpolation = CV_INTER_LINEAR) const;

  void rectifyPoint(const Eigen::Vector2d& uv_raw, Eigen::Vector2d& uv_rect) const ROS_DEPRECATED;
  
  /**
   * \brief Compute the rectified image coordinates of a pixel in the raw image.
   */
  Eigen::Vector2d rectifyPoint(const Eigen::Vector2d& uv_raw) const;

  void unrectifyPoint(const Eigen::Vector2d& uv_rect, Eigen::Vector2d& uv_raw) const ROS_DEPRECATED;
  
  /**
   * \brief Compute the raw image coordinates of a pixel in the rectified image.
   */
  Eigen::Vector2d unrectifyPoint(const Eigen::Vector2d& uv_rect) const;

  /**
   * \brief Compute the rectified ROI best fitting a raw ROI.
   */
  cv::Rect rectifyRoi(const cv::Rect& roi_raw) const;

  /**
   * \brief Compute the raw ROI best fitting a rectified ROI.
   */
  cv::Rect unrectifyRoi(const cv::Rect& roi_rect) const;
  
  /**
   * \brief Returns the original camera matrix.
   */
  const Eigen::Matrix3d& inputIntrinsicMatrix() const;

  /**
   * \brief Returns the original camera matrix.
   */
  const Eigen::Matrix3d& outputIntrinsicMatrix() const;

  /**
   * \brief Returns the distortion coefficients.
   */
  const Eigen::VectorXd& distortionCoeffs() const;

  /**
   * \brief Returns the distortion coefficients.
   */
  const Eigen::Matrix<double,5,1>& rationalCoeffs() const;

  /**
   * \brief Returns the rotation matrix.
   */
  const Eigen::Matrix3d& rotationMatrix() const;

  /**
   * \brief Returns the original camera matrix for full resolution.
   */
  const Eigen::Matrix3d& inputFullIntrinsicMatrix() const;

  /**
   * \brief Returns the focal length (pixels) in x direction of the rectified image.
   */
  double fx() const;

  /**
   * \brief Returns the focal length (pixels) in y direction of the rectified image.
   */
  double fy() const;

  /**
   * \brief Returns the x coordinate of the optical center.
   */
  double cx() const;

  /**
   * \brief Returns the y coordinate of the optical center.
   */
  double cy() const;

  /**
   * \brief Returns the image height.
   */
  int height() const ROS_DEPRECATED;

  /**
   * \brief Returns the image width.
   */
  int width() const ROS_DEPRECATED;

  /**
   * \brief Returns the number of columns in each bin.
   */
  int binningX() const;

  /**
   * \brief Returns the number of rows in each bin.
   */
  int binningY() const;
  
  /**
   * \brief Compute delta u, given Z and delta X in Cartesian space.
   *
   * For given Z, this is the inverse of getDeltaX().
   *
   * \param deltaX Delta X, in Cartesian space
   * \param Z      Z (depth), in Cartesian space
   */
  double getDeltaU(double deltaX, double Z) const;

  /**
   * \brief Compute delta v, given Z and delta Y in Cartesian space.
   *
   * For given Z, this is the inverse of getDeltaY().
   *
   * \param deltaY Delta Y, in Cartesian space
   * \param Z      Z (depth), in Cartesian space
   */
  double getDeltaV(double deltaY, double Z) const;

  /**
   * \brief Compute delta X, given Z in Cartesian space and delta u in pixels.
   *
   * For given Z, this is the inverse of getDeltaU().
   *
   * \param deltaU Delta u, in pixels
   * \param Z      Z (depth), in Cartesian space
   */
  double getDeltaX(double deltaU, double Z) const;

  /**
   * \brief Compute delta Y, given Z in Cartesian space and delta v in pixels.
   *
   * For given Z, this is the inverse of getDeltaV().
   *
   * \param deltaV Delta v, in pixels
   * \param Z      Z (depth), in Cartesian space
   */
  double getDeltaY(double deltaV, double Z) const;
  
  /**
   * \brief Read a calibration file in YAML format
   *
   *
   * \param filename Name of the file to read
   */
  void readCalibration(std::string filename);

  /**
   * \brief Write a calibration file in YAML format
   *
   *
   * \param filename Name of the file to read
   */
  void writeCalibration(std::string filename) const;

  Eigen::Matrix3d getK()const { return K_;}

  Eigen::Matrix3d getKp()const { return Kp_;}

  Eigen::VectorXd getD() const { return D_;}
protected:
  int width_, height_;               // camera resolution
  int binning_x_, binning_y_;        // image reduction
  Eigen::VectorXd D_;                // Distortion coefficients: k1, k2, t1, t2, k3 for Tsai model (5x1),
                                     //  8x1 for rational model
  Eigen::Matrix3d R_;                // Rotation matrix, from input to output image
  Eigen::Matrix3d K_;                // Input image camera internals
  Eigen::Matrix3d K_full_;           // Input image camera internals, full image (no binning or ROI)
  Eigen::Matrix3d Kp_;               // Output image camera internals
  Eigen::Matrix3d Kp_full_;          // Output image camera internals, full image
  cv::Rect rect_roi_, input_roi_;    // Region of interest for image
  double cx_offset_, cy_offset_;     // Offset of input image from calibration image
  cv::Mat rtemp_;                    // Temporary image for computation

  void initRectificationMaps() const;

  // Use PIMPL here so we can change internals in patch updates if needed
  struct Cache;
  boost::shared_ptr<Cache> cache_; // Holds cached data for internal use
  bool initialized() const { return cache_; }

  friend class StereoCameraModel;
};


/* Trivial inline functions */

inline const Eigen::Matrix3d& PinholeCameraModel::inputIntrinsicMatrix() const  { return K_; }
inline const Eigen::Matrix3d& PinholeCameraModel::inputFullIntrinsicMatrix() const  { return K_full_; }
inline const Eigen::Matrix3d& PinholeCameraModel::outputIntrinsicMatrix() const  { return Kp_; }
inline const Eigen::VectorXd& PinholeCameraModel::distortionCoeffs() const { return D_; }
inline const Eigen::Matrix3d& PinholeCameraModel::rotationMatrix() const   { return R_; }

inline double PinholeCameraModel::fx() const { return Kp_(0,0); } 
inline double PinholeCameraModel::fy() const { return Kp_(1,1); }
inline double PinholeCameraModel::cx() const { return Kp_(0,2); }
inline double PinholeCameraModel::cy() const { return Kp_(1,2); }
inline int    PinholeCameraModel::height() const { return height_; }
inline int    PinholeCameraModel::width() const  { return width_; }

inline int    PinholeCameraModel::binningX() const { return binning_x_; }
inline int    PinholeCameraModel::binningY() const { return binning_y_; }

inline double PinholeCameraModel::getDeltaU(double deltaX, double Z) const
{
  assert( initialized() );
  return fx() * deltaX / Z;
}

inline double PinholeCameraModel::getDeltaV(double deltaY, double Z) const
{
  assert( initialized() );
  return fy() * deltaY / Z;
}

inline double PinholeCameraModel::getDeltaX(double deltaU, double Z) const
{
  assert( initialized() );
  return Z * deltaU / fx();
}

inline double PinholeCameraModel::getDeltaY(double deltaV, double Z) const
{
  assert( initialized() );
  return Z * deltaV / fy();
}

} //namespace image_geometry

#endif
