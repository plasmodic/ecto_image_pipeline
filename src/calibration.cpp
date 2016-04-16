#include <opencv2/calib3d/calib3d.hpp>

#include <ecto_image_pipeline/calibration.hpp>
#include <ecto_image_pipeline/conversions.hpp>
#include <ecto_image_pipeline/image_pipeline.hpp>

namespace image_pipeline
{
  void
  calibrate_stereo(const observation_pts_v_t& left_points, const observation_pts_v_t& right_points,
                   const object_pts_v_t& object_points, const cv::Size& image_size, PinholeCameraModel& left,
                   PinholeCameraModel& right)
  {
    PinholeCameraModel camera;
    StereoCameraModel scam;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat K_left, K_right, D_right, D_left, R, T, E, F;
    int flags =cv::CALIB_FIX_PRINCIPAL_POINT| cv::CALIB_FIX_ASPECT_RATIO /*| cv::CALIB_ZERO_TANGENT_DIST|cv::CALIB_FIX_PRINCIPAL_POINT| cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 |cv::CALIB_FIX_K3*/;
    cv::Size left_size(image_size), right_size(image_size);
    double left_rms = cv::calibrateCamera(object_points, left_points, left_size, K_left, D_left, rvecs, tvecs, flags);
    double right_rms = cv::calibrateCamera(object_points, right_points, right_size, K_right, D_right, rvecs, tvecs,
                                           flags);

#if CV_MAJOR_VERSION >= 3
    double stereo_rms = cv::stereoCalibrate(object_points, left_points, right_points, K_left, D_left, K_right, D_right,
                                            left_size, R, T, E, F,
                                            cv::CALIB_FIX_INTRINSIC | flags,
                                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, 1e-6));
#else
    double stereo_rms = cv::stereoCalibrate(object_points, left_points, right_points, K_left, D_left, K_right, D_right,
                                            left_size, R, T, E, F,
                                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, 1e-6),
                                            cv::CALIB_FIX_INTRINSIC | flags);
#endif

    std::cout << "image size:" << image_size.width << ":" << image_size.height << std::endl;
    std::cout << "R:" << R << std::endl;
    std::cout << "T:" << T << std::endl;
    std::cout << "D:" << D_left << std::endl;
    std::cout << "left rms:" << left_rms << " right rms:" << right_rms << " stereo rms:" << stereo_rms << std::endl;
    left.setParams(left_size, K_left, D_left, cv::Mat_<double>::eye(3, 3), K_left);
    right.setParams(right_size, K_right, D_right, cv::Mat_<double>::eye(3, 3), K_right);
    left.writeCalibration("left.yml");
    right.writeCalibration("right.yml");
    Eigen::Matrix3d Re;
    Eigen::Vector3d Te;
    Eigen::Matrix4d Pe = Eigen::Matrix4d::Identity();
    cv2eigen(R, Re);
    cv2eigen(T.t(), Te);
    std::cout << Te << std::endl;
    Pe.block(0, 0, 3, 3) = Re;
    Pe.block(0, 3, 3, 1) = Te;
    Pose P;
    P.transform.matrix() = Pe;
    scam.setParams(left, right, P);
    scam.writeCalibration("stereo.yml");
  }

}
