#include <opencv2/calib3d.hpp>
namespace srrg2_apriltag_calibration {

  template <APRILTAG_FAMILY apriltag_family_>
  void Apriltag_<apriltag_family_>::computeTagPose(srrg2_core::Isometry3f& tag_relative_pose_,
                                                   const float& tag_size_,
                                                   const srrg2_core::Matrix3f& camera_matrix_,
                                                   const srrg2_core::Vector4f& distortion_coeffs_,
                                                   const bool do_ransac_) const {
    const size_t& num_corners  = _corners.size();
    const float half_tag_size_ = tag_size_ / 2.f;
    assert(num_corners == NumTagCorners &&
           "Apriltag_::computeTagPose|ERROR, unexpected number of tag coners");

    std::vector<cv::Point3f> cv_world_points(num_corners); // ia landmarks
    std::vector<cv::Point2f> cv_image_points(num_corners); // ia observations

    // ia fill world points with the standard tag {1/-1, 1/-1} scaled by the tag_size
    cv_world_points[0] = cv::Point3f(-half_tag_size_, -half_tag_size_, 0);
    cv_world_points[1] = cv::Point3f(half_tag_size_, -half_tag_size_, 0);
    cv_world_points[2] = cv::Point3f(half_tag_size_, half_tag_size_, 0);
    cv_world_points[3] = cv::Point3f(-half_tag_size_, half_tag_size_, 0);

    // ia copy observations in opencv things
    for (size_t i = 0; i < num_corners; ++i) {
      const srrg2_core::Vector2f& corner = _corners[i];
      cv_image_points[i]                 = cv::Point2f(corner.x(), corner.y());
    }

    // ia copy camera matrix in opencv things
    cv::Matx33f cv_camera_matrix = cv::Matx33f::zeros();
    for (int r = 0; r < camera_matrix_.rows(); ++r) {
      for (int c = 0; c < camera_matrix_.cols(); ++c) {
        cv_camera_matrix(r, c) = camera_matrix_(r, c);
      }
    }

    // ia copy distortion coefficients
    cv::Vec4f cv_distortion_coeffs;
    for (int i = 0; i < distortion_coeffs_.rows(); ++i) {
      cv_distortion_coeffs(i) = distortion_coeffs_(i);
    }

    // ia setup cv PnP solver
    cv::Mat cv_translation, cv_rotation_vector;
    cv::Matx33d cv_R;
    if (do_ransac_) {
      cv::solvePnPRansac(cv_world_points,
                         cv_image_points,
                         cv_camera_matrix,
                         cv_distortion_coeffs,
                         cv_translation,
                         cv_rotation_vector);
    } else {
      cv::solvePnP(cv_world_points,
                   cv_image_points,
                   cv_camera_matrix,
                   cv_distortion_coeffs,
                   cv_translation,
                   cv_rotation_vector);
    }

    cv::Rodrigues(cv_rotation_vector, cv_R);

    // ia copy in eigen friendly data the result
    srrg2_core::Matrix3f rotation    = srrg2_core::Matrix3f::Identity();
    srrg2_core::Vector3f translation = srrg2_core::Vector3f::Zero();
    for (int r = 0; r < rotation.rows(); ++r) {
      for (int c = 0; c < rotation.cols(); ++c) {
        rotation(r, c) = cv_R((int) r, (int) c);
      }
      translation(r) = cv_translation.at<float>((int) r);
    }

    tag_relative_pose_.linear()      = rotation;
    tag_relative_pose_.translation() = translation;
  }

  template <APRILTAG_FAMILY apriltag_family_>
  void Apriltag_<apriltag_family_>::draw(cv::Mat& cv_bgr_image_) const {
    assert(_corners.size() == NumTagCorners &&
           "Apriltag_::draw|ERROR, unexpected number of tag coners");
    // ia cache corners
    const srrg2_core::Vector2f& c0 = _corners[0];
    const srrg2_core::Vector2f& c1 = _corners[1];
    const srrg2_core::Vector2f& c2 = _corners[2];
    const srrg2_core::Vector2f& c3 = _corners[3];

    // ia draw corners
    cv::line(cv_bgr_image_,
             cv::Point(c0.x(), c0.y()),
             cv::Point(c1.x(), c1.y()),
             cv::Scalar(0, 0xff, 0),
             2);
    cv::line(cv_bgr_image_,
             cv::Point(c0.x(), c0.y()),
             cv::Point(c3.x(), c3.y()),
             cv::Scalar(0, 0, 0xff),
             2);
    cv::line(cv_bgr_image_,
             cv::Point(c1.x(), c1.y()),
             cv::Point(c2.x(), c2.y()),
             cv::Scalar(0xff, 0, 0),
             2);
    cv::line(cv_bgr_image_,
             cv::Point(c2.x(), c2.y()),
             cv::Point(c3.x(), c3.y()),
             cv::Scalar(0xff, 0, 0),
             2);

    // ia show also corner number
    for (size_t k = 0; k < _corners.size(); ++k) {
      std::string text = std::to_string(_tag_id) + "." + std::to_string(k);
      cv::Point2f text_center(_corners[k].x() + 10, _corners[k].y());
      cv::putText(
        cv_bgr_image_, text, text_center, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
    }

    // ia draw center
    cv::circle(
      cv_bgr_image_, cv::Point2f(_center.x(), _center.y()), 2, cv::Scalar(0, 0, 255, 0), 2);

    // ia show ID
    cv::putText(cv_bgr_image_,
                std::to_string(_tag_id) + ".5",
                cv::Point2f(_center.x() + 10, _center.y()),
                cv::FONT_HERSHEY_PLAIN,
                1,
                cv::Scalar(0, 0, 255));
  }

} // namespace srrg2_apriltag_calibration
