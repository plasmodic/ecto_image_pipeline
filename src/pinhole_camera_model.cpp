#include <exception>
#include <iostream>

#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <ecto_image_pipeline/pinhole_camera_model.h>


namespace image_pipeline {

enum DistortionState { NONE, CALIBRATED, UNKNOWN };

struct PinholeCameraModel::Cache
{
  DistortionState distortion_state;

  cv::Mat_<double> K_binned, Kp_binned; // Binning applied, but not cropping

  mutable bool full_maps_dirty;
  mutable cv::Mat full_map1, full_map2;

  mutable bool reduced_maps_dirty;
  mutable cv::Mat reduced_map1, reduced_map2;

  mutable bool rectified_roi_dirty;
  mutable cv::Rect rectified_roi;

  Cache()
    : full_maps_dirty(true),
      reduced_maps_dirty(true),
      rectified_roi_dirty(true)
  {
  }
};

PinholeCameraModel::PinholeCameraModel()
{
  // Create our repository of cached data (rectification maps, etc.)
  cache_ = boost::make_shared<Cache>();
  cache_->distortion_state = NONE;
}

// inequality operator
bool PinholeCameraModel::operator != (const PinholeCameraModel& mCamObj)
{
  if (mCamObj.K_ != K_ ||
      mCamObj.R_ != R_ ||
      mCamObj.D_ != D_ ||
      mCamObj.Kp_ != Kp_ ||
      mCamObj.width_ != width_ ||
      mCamObj.height_ != height_ 
      )
    return true;
  return false;
}

// clear the cache and make it local
void PinholeCameraModel::initCache()
{
  // Create our repository of cached data (rectification maps, etc.)
  cache_ = boost::make_shared<Cache>();
  cache_->distortion_state = (D_.size() > 0 && D_(0) == 0.0) ? NONE : CALIBRATED;
}


// initialize the camera model
void
PinholeCameraModel::setParams(cv::Size &size, Eigen::Matrix3d &K, Eigen::VectorXd &D, Eigen::Matrix3d &R, Eigen::Matrix3d &Kp, const double ox, const double oy)
{
  width_ = size.width;
  height_ = size.height;
  R_ = R;
  D_ = D;
  K_ = K;
  K_full_ = K;
  Kp_ = Kp;
  Kp_full_ = Kp;
  binning_x_ = 1;
  binning_y_ = 1;
  cx_offset_ = ox;
  cy_offset_ = oy;

  cache_->distortion_state = (D_.size() > 0 && D_(0) == 0.0) ? NONE : CALIBRATED;
  cache_->full_maps_dirty;
}

void
PinholeCameraModel::setParams(cv::Size image_size, const cv::Mat& K, const cv::Mat& D, const cv::Mat& R,
                              const cv::Mat&Kp, const double ox, const double oy)
{
  Eigen::Matrix3d eK, eKp, eR;
  Eigen::VectorXd eD;
  cv::cv2eigen(K, eK);
  if (D.empty())
  {
    eD.setZero();
  }
  else
  {
    if(D.rows == 1)
    {
      cv2eigen(D.t(), eD);
    }else
    {
      cv2eigen(D, eD);
    }
  }

  if (R.empty())
    R_.setIdentity();
  else
    cv::cv2eigen(R, eR);

  if (Kp.empty())
    eKp = eK;
  else
    cv2eigen(Kp, eKp);
  setParams(image_size, eK, eD, eR, eKp, ox, oy);
}


void 
PinholeCameraModel::setCenterOffset(const double ox, const double oy)
{
  cx_offset_ = ox;
  cy_offset_ = oy;
  cache_->full_maps_dirty = true;
}


void PinholeCameraModel::toCv(cv::Size& size,cv::Mat& K,  cv::Mat& D, cv::Mat& R,
                               cv::Mat&Kp)
{
  size.width = width_;
  size.height = height_;
  cv::eigen2cv(K_,K);
  cv::eigen2cv(D_,D);
  cv::eigen2cv(R_,R);
  cv::eigen2cv(Kp_,Kp);
}

// For uint32_t, string, bool...
template<typename T>
bool update(const T& new_val, T& my_val)
{
  if (my_val == new_val)
    return false;
  my_val = new_val;
  return true;
}

// For boost::array, std::vector
template<typename MatT>
bool updateMat(const MatT& new_mat, MatT& my_mat, cv::Mat_<double>& cv_mat, int rows, int cols)
{
  if (my_mat == new_mat)
    return false;
  my_mat = new_mat;
  // D may be empty if camera is uncalibrated or distortion model is non-standard&
  cv_mat = (my_mat.size() == 0) ? cv::Mat_<double>() : cv::Mat_<double>(rows, cols, &my_mat[0]);
  return true;
}

void PinholeCameraModel::project3dToPixel(const Eigen::Vector3d& xyz, Eigen::Vector2d& uv_rect) const
{
  uv_rect = project3dToPixel(xyz);
}

cv::Size PinholeCameraModel::fullResolution() const
{
  assert( initialized() );
  return cv::Size(width_, height_);
}

cv::Size PinholeCameraModel::reducedResolution() const
{
  assert( initialized() );

  cv::Rect roi = rectifiedRoi();
  return cv::Size(roi.width / binningX(), roi.height / binningY());
}

Eigen::Vector2d PinholeCameraModel::toFullResolution(const Eigen::Vector2d& uv_reduced) const
{
  cv::Rect roi = rectifiedRoi();
  return Eigen::Vector2d(uv_reduced.x() * binningX() + roi.x,
                         uv_reduced.y() * binningY() + roi.y);
}

cv::Rect PinholeCameraModel::toFullResolution(const cv::Rect& roi_reduced) const
{
  cv::Rect roi = rectifiedRoi();
  return cv::Rect(roi_reduced.x * binningX() + roi.x,
                  roi_reduced.y * binningY() + roi.y,
                  roi_reduced.width  * binningX(),
                  roi_reduced.height * binningY());
}

Eigen::Vector2d PinholeCameraModel::toReducedResolution(const Eigen::Vector2d& uv_full) const
{
  cv::Rect roi = rectifiedRoi();
  return Eigen::Vector2d((uv_full.x() - roi.x) / binningX(),
                     (uv_full.y() - roi.y) / binningY());
}

cv::Rect PinholeCameraModel::toReducedResolution(const cv::Rect& roi_full) const
{
  cv::Rect roi = rectifiedRoi();
  return cv::Rect((roi_full.x - roi.x) / binningX(),
                  (roi_full.y - roi.y) / binningY(),
                  roi_full.width  / binningX(),
                  roi_full.height / binningY());
}

cv::Rect PinholeCameraModel::rawRoi() const
{
  assert( initialized() );

  return cv::Rect(0,0,width_,height_);
  //  return cv::Rect(cam_info_.roi.x()_offset, cam_info_.roi.y_offset,
  //                  cam_info_.roi.width, cam_info_.roi.height);
}

cv::Rect PinholeCameraModel::rectifiedRoi() const
{
  assert( initialized() );

  if (cache_->rectified_roi_dirty)
  {
    //    if (!cam_info_.roi.do_rectify)
    //      cache_->rectified_roi = rawRoi();
    //    else
    //      cache_->rectified_roi = rectifyRoi(rawRoi());
    //    cache_->rectified_roi_dirty = false;
  }
  return cache_->rectified_roi;
}

Eigen::Vector2d PinholeCameraModel::project3dToPixel(const Eigen::Vector3d& xyz) const
{
  assert( initialized() );

  // [U V W]^T = P * [X Y Z 1]^T
  // u = U/W
  // v = V/W
  Eigen::Vector2d uv_rect;
  uv_rect.x() = (fx()*xyz.x()) / xyz.z() + cx();
  uv_rect.y() = (fy()*xyz.y()) / xyz.z() + cy();
  return uv_rect;
}

void PinholeCameraModel::projectPixelTo3dRay(const Eigen::Vector2d& uv_rect, Eigen::Vector3d& ray) const
{
  ray = projectPixelTo3dRay(uv_rect);
}

Eigen::Vector3d PinholeCameraModel::projectPixelTo3dRay(const Eigen::Vector2d& uv_rect) const
{
  assert( initialized() );

  Eigen::Vector3d ray;
  ray.x() = (uv_rect.x() - cx()) / fx();
  ray.y() = (uv_rect.y() - cy()) / fy();
  ray.z() = 1.0;
  return ray;
}

void PinholeCameraModel::rectifyImage(const cv::Mat& raw, cv::Mat& rectified, int interpolation) const
{
  assert( initialized() );

  switch (cache_->distortion_state) {
    case NONE:
      raw.copyTo(rectified);
      break;
    case CALIBRATED:
      initRectificationMaps();
      cv::remap(raw, rectified, cache_->reduced_map1, cache_->reduced_map2, interpolation);
      break;
    default:
      assert(cache_->distortion_state == UNKNOWN);
      throw std::runtime_error("Cannot call rectifyImage when distortion is unknown.");
  }
}


void PinholeCameraModel::unrectifyImage(const cv::Mat& rectified, cv::Mat& raw, int interpolation) const
{
  assert( initialized() );

  throw std::runtime_error("PinholeCameraModel::unrectifyImage is unimplemented.");
  /// @todo Implement unrectifyImage()
  // Similar to rectifyImage, but need to build separate set of inverse maps (raw->rectified)...
  // - Build src_pt Mat with all the raw pixel coordinates (or do it one row at a time)
  // - Do cv::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_)
  // - Use convertMaps() to convert dst_pt to fast fixed-point maps
  // - cv::remap(rectified, raw, ...)
  // Need interpolation argument. Same caching behavior?
}

void PinholeCameraModel::rectifyPoint(const Eigen::Vector2d& uv_raw, Eigen::Vector2d& uv_rect) const
{
  uv_rect = rectifyPoint(uv_raw);
}

Eigen::Vector2d PinholeCameraModel::rectifyPoint(const Eigen::Vector2d& uv_raw) const
{
  assert( initialized() );

  if (cache_->distortion_state == NONE)
    return uv_raw;
  if (cache_->distortion_state == UNKNOWN)
    throw std::runtime_error("Cannot call rectifyPoint when distortion is unknown.");
  assert(cache_->distortion_state == CALIBRATED);

  // convert Eigen matrix to OpenCV matrix
  cv::Mat K, Kp, R, D;
  cv::eigen2cv(K_,K);
  cv::eigen2cv(D_,D);
  cv::eigen2cv(Kp_,Kp);
  cv::eigen2cv(R_,R);

  /// @todo cv::undistortPoints requires the point data to be float, should allow double
  Eigen::Vector2d rect;
  const cv::Mat src_pt(1, 1, CV_64FC2, (void *)uv_raw.data());
  cv::Mat dst_pt(1, 1, CV_64FC2, (void *)rect.data());
  cv::undistortPoints(src_pt, dst_pt, K, D, R, Kp);
  return rect;
}


void PinholeCameraModel::unrectifyPoint(const Eigen::Vector2d& uv_rect, Eigen::Vector2d& uv_raw) const
{
  uv_raw = unrectifyPoint(uv_rect);
}

Eigen::Vector2d PinholeCameraModel::unrectifyPoint(const Eigen::Vector2d& uv_rect) const
{
  assert( initialized() );

  if (cache_->distortion_state == NONE)
    return uv_rect;
  if (cache_->distortion_state == UNKNOWN)
    throw std::runtime_error("Cannot call unrectifyPoint when distortion is unknown.");
  assert(cache_->distortion_state == CALIBRATED);

  /// @todo Make this just call projectPixelTo3dRay followed by cv::projectPoints. But
  /// cv::projectPoints requires 32-bit float, which is annoying.

  // Formulae from docs for cv::initUndistortRectifyMap,
  // http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html

  // x <- (u - c'x) / f'x
  // y <- (v - c'y) / f'y
  // c'x, f'x, etc. (primed) come from "new camera matrix" P[0:3, 0:3].
  double x = (uv_rect.x() - cx()) / fx();
  double y = (uv_rect.y() - cy()) / fy();
  // [X Y W]^T <- R^-1 * [x y 1]^T
  double X = R_(0,0)*x + R_(1,0)*y + R_(2,0);
  double Y = R_(0,1)*x + R_(1,1)*y + R_(2,1);
  double W = R_(0,2)*x + R_(1,2)*y + R_(2,2);
  // x' <- X/W, y' <- Y/W
  double xp = X / W;
  double yp = Y / W;
  // x'' <- x'(1+k1*r^2+k2*r^4+k3*r^6) + 2p1*x'*y' + p2(r^2+2x'^2)
  // y'' <- y'(1+k1*r^2+k2*r^4+k3*r^6) + p1(r^2+2y'^2) + 2p2*x'*y'
  // where r^2 = x'^2 + y'^2
  double r2 = xp*xp + yp*yp;
  double r4 = r2*r2;
  double r6 = r4*r2;
  double a1 = 2*xp*yp;
  double k1 = D_(0,0), k2 = D_(0,1), p1 = D_(0,2), p2 = D_(0,3), k3 = D_(0,4);
  double barrel_correction = 1 + k1*r2 + k2*r4 + k3*r6;
  if (D_.cols() == 8)
    barrel_correction /= (1.0 + D_(0,5)*r2 + D_(0,6)*r4 + D_(0,7)*r6);
  double xpp = xp*barrel_correction + p1*a1 + p2*(r2+2*(xp*xp));
  double ypp = yp*barrel_correction + p1*(r2+2*(yp*yp)) + p2*a1;
  // map_x(u,v) <- x''fx + cx
  // map_y(u,v) <- y''fy + cy
  // cx, fx, etc. come from original camera matrix K.
  return Eigen::Vector2d(xpp*K_(0,0) + K_(0,2), ypp*K_(1,1) + K_(1,2));
}

cv::Rect PinholeCameraModel::rectifyRoi(const cv::Rect& roi_raw) const
{
  assert( initialized() );

  /// @todo Actually implement "best fit" as described by REP 104.

  // For now, just unrectify the four corners and take the bounding box.
  Eigen::Vector2d rect_tl = rectifyPoint(Eigen::Vector2d(roi_raw.x, roi_raw.y));
  Eigen::Vector2d rect_tr = rectifyPoint(Eigen::Vector2d(roi_raw.x + roi_raw.width, roi_raw.y));
  Eigen::Vector2d rect_br = rectifyPoint(Eigen::Vector2d(roi_raw.x + roi_raw.width,
                                                 roi_raw.y + roi_raw.height));
  Eigen::Vector2d rect_bl = rectifyPoint(Eigen::Vector2d(roi_raw.x, roi_raw.y + roi_raw.height));

  cv::Point roi_tl(std::ceil (std::min(rect_tl.x(), rect_bl.x())),
                   std::ceil (std::min(rect_tl.y(), rect_tr.y())));
  cv::Point roi_br(std::floor(std::max(rect_tr.x(), rect_br.x())),
                   std::floor(std::max(rect_bl.y(), rect_br.y())));

  return cv::Rect(roi_tl.x, roi_tl.y, roi_br.x - roi_tl.x, roi_br.y - roi_tl.y);
}

cv::Rect PinholeCameraModel::unrectifyRoi(const cv::Rect& roi_rect) const
{
  assert( initialized() );

  /// @todo Actually implement "best fit" as described by REP 104.

  // For now, just unrectify the four corners and take the bounding box.
  Eigen::Vector2d raw_tl = unrectifyPoint(Eigen::Vector2d(roi_rect.x, roi_rect.y));
  Eigen::Vector2d raw_tr = unrectifyPoint(Eigen::Vector2d(roi_rect.x + roi_rect.width, roi_rect.y));
  Eigen::Vector2d raw_br = unrectifyPoint(Eigen::Vector2d(roi_rect.x + roi_rect.width,
                                                  roi_rect.y + roi_rect.height));
  Eigen::Vector2d raw_bl = unrectifyPoint(Eigen::Vector2d(roi_rect.x, roi_rect.y + roi_rect.height));

  cv::Point roi_tl(std::floor(std::min(raw_tl.x(), raw_bl.x())),
                   std::floor(std::min(raw_tl.y(), raw_tr.y())));
  cv::Point roi_br(std::ceil (std::max(raw_tr.x(), raw_br.x())),
                   std::ceil (std::max(raw_bl.y(), raw_br.y())));

  return cv::Rect(roi_tl.x, roi_tl.y, roi_br.x - roi_tl.x, roi_br.y - roi_tl.y);
}

void PinholeCameraModel::initRectificationMaps() const
{
  /// @todo For large binning settings, can drop extra rows/cols at bottom/right boundary.
  /// Make sure we're handling that 100% correctly.

  if (cache_->full_maps_dirty) {
    // Create the full-size map at the binned resolution
    /// @todo Should binned resolution, K, P be part of public API?
    cv::Size binned_resolution = fullResolution();
    binned_resolution.width  /= binningX();
    binned_resolution.height /= binningY();

    Eigen::Matrix3d K_binned, Kp_binned, Km;

    // offset orig image K if necessary (for depth images that have x,y offset)
    Km = K_;
    Km(0,2) += cx_offset_;
    Km(1,2) += cy_offset_;

    if (binningX() == 1 && binningY() == 1) {
      K_binned = K_full_;
      Kp_binned = Kp_full_;
    }
    else {
      K_binned = K_full_;
      Kp_binned = Kp_full_;
      if (binningX() > 1) {
        double scale_x = 1.0 / binningX();
        K_binned(0,0) *= scale_x;
        K_binned(0,2) *= scale_x;
        Kp_binned(0,0) *= scale_x;
        Kp_binned(0,2) *= scale_x;
      }
      if (binningY() > 1) {
        double scale_y = 1.0 / binningY();
        K_binned(1,1) *= scale_y;
        K_binned(1,2) *= scale_y;
        Kp_binned(1,1) *= scale_y;
        Kp_binned(1,2) *= scale_y;
      }
    }

    // Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
    cv::Mat K, Kp, R, D;
    cv::eigen2cv(Km,K);
    cv::eigen2cv(D_,D);
    cv::eigen2cv(Kp_,Kp);
    cv::eigen2cv(R_,R);

    //    std::cout << Km << std::endl;
    //    std::cout << R << std::endl;
    //    std::cout << D << std::endl << std::endl;

    cv::initUndistortRectifyMap(K, D, R, Kp, binned_resolution,
                                CV_16SC2, cache_->full_map1, cache_->full_map2);
    cache_->full_maps_dirty = false;
  }

  if (cache_->reduced_maps_dirty) {
    /// @todo Use rectified ROI
    //    cv::Rect roi(cam_info_.roi.x_offset, cam_info_.roi.y_offset,
    //                 cam_info_.roi.width, cam_info_.roi.height);
    cv::Rect roi(0,0,width_,height_);
    if (roi.x != 0 || roi.y != 0 ||
        roi.height != (int)height_ ||
        roi.width  != (int)width_) {

      // map1 contains integer (x,y) offsets, which we adjust by the ROI offset
      // map2 contains LUT index for subpixel interpolation, which we can leave as-is
      roi.x /= binningX();
      roi.y /= binningY();
      roi.width  /= binningX();
      roi.height /= binningY();
      cache_->reduced_map1 = cache_->full_map1(roi) - cv::Scalar(roi.x, roi.y);
      cache_->reduced_map2 = cache_->full_map2(roi);
    }
    else {
      // Otherwise we're rectifying the full image
      cache_->reduced_map1 = cache_->full_map1;
      cache_->reduced_map2 = cache_->full_map2;
    }
    cache_->reduced_maps_dirty = false;
  }
}

void PinholeCameraModel::readCalibration(std::string calibfile)
{
  cv::FileStorage fs(calibfile, cv::FileStorage::READ);
  CV_Assert(fs.isOpened());
  cv::Mat K,Kp,R,D,P;
  int width, height;
  cv::read(fs["input_camera_matrix"], K, cv::Mat());
  cv::read(fs["distortion_coefficients"], D, cv::Mat());
  cv::read(fs["rotation_matrix"], R, cv::Mat());
  cv::read(fs["rectified_camera_matrix"], Kp, cv::Mat());
  cv::read(fs["image_width"], width, 0);
  cv::read(fs["image_height"], height, 0);
  CV_Assert(K.empty() == false);
  setParams(cv::Size(width,height),K,D,R,Kp);
}


//
// Store all the parameters
//
void PinholeCameraModel::writeCalibration(std::string calibfile) const
{
  cv::FileStorage fs(calibfile, cv::FileStorage::WRITE);
  CV_Assert(fs.isOpened());
  cv::Mat K,Kp,R,D,P;
  cv::eigen2cv(K_,K);
  cv::eigen2cv(D_,D);
  cv::eigen2cv(Kp_,Kp);
  cv::eigen2cv(R_,R);
  cvWriteComment(*fs, "Camera", 0);

  fs << "image_width" << width_;
  fs << "image_height" << height_;

  if (!K.empty())
    fs << "input_camera_matrix" << K;
  if (!D.empty())
    fs << "distortion_coefficients" << D;
  if (!R.empty())
    fs << "rotation_matrix" << R;
  if (!Kp.empty())
    fs << "rectified_camera_matrix" << Kp;
}


} //namespace image_geometry
