#include <image_pipeline/calibration.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_pipeline/image_pipeline.hpp>
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
    int flags = CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_ZERO_TANGENT_DIST;
    cv::Size left_size(image_size), right_size(image_size);

    double left_rms = cv::calibrateCamera(object_points, left_points, left_size, K_left, D_left, rvecs, tvecs, flags);
    double right_rms = cv::calibrateCamera(object_points, right_points, right_size, K_right, D_right, rvecs, tvecs,
                                           flags);
    double stereo_rms = cv::stereoCalibrate(object_points, left_points, right_points, K_left, D_left, K_right, D_right,
                                            left_size, R, T, E, F,
                                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6),
                                            cv::CALIB_FIX_INTRINSIC);

    std::cout << "image size:" << image_size.width << ":" << image_size.height << std::endl;
    std::cout << "R:" << R << std::endl;
    std::cout << "T:" << T << std::endl;
    std::cout << "left rms:" << left_rms << " right rms:" << right_rms << " stereo rms:" << stereo_rms << std::endl;
    left.setParams(left_size, K_left, D_left, cv::Mat_<double>::eye(3, 3), K_left);
    right.setParams(right_size, K_right, D_right, R, K_right);
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
    P.transform = Pe;
    scam.setParams(left, right, P);
    scam.writeCalibration("stereo.yml");
  }

}
