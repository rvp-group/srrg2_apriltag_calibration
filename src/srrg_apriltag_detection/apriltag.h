#pragma once
#include <map>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "apriltag3/apriltag.h"
#include "apriltag3/tag36h11.h"

#include <srrg_geometry/geometry_defs.h>
namespace srrg2_apriltag_calibration {

  //! @brief apriltag family
  enum APRILTAG_FAMILY { Invalid = 0x00, Family_36h11 = 0x01 };

  //! @brief wrapper to the apriltag detection structure of the AprilTag3 library
  template <APRILTAG_FAMILY apriltag_family_>
  class Apriltag_ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief some usings
    static constexpr APRILTAG_FAMILY ApriltagFamily = apriltag_family_;
    static constexpr size_t NumTagCorners           = 4;
    using Vector2fVector =
      std::vector<srrg2_core::Vector2f, Eigen::aligned_allocator<srrg2_core::Vector2f>>;

    //! @brief object life
    Apriltag_() {
      _corners.clear();
    }

    Apriltag_(apriltag_detection* at_detection_) {
      _corners.clear();
      _tag_id           = at_detection_->id;
      _hamming_distance = at_detection_->hamming;
      _decision_margin  = at_detection_->decision_margin;

      // ia copy homography
      for (int r = 0; r < _homography.rows(); ++r) {
        for (int c = 0; c < _homography.cols(); ++c) {
          _homography(r, c) = matd_get(at_detection_->H, r, c);
        }
      }

      // ia copy center
      _center << at_detection_->c[0], at_detection_->c[1];

      // ia copy corners
      _corners.reserve(NumTagCorners);
      for (size_t i = 0; i < NumTagCorners; ++i) {
        srrg2_core::Vector2f corner(at_detection_->p[i][0], at_detection_->p[i][1]);
        _corners.emplace_back(corner);
      }
    }

    ~Apriltag_() = default;

    //! @brief const accessor
    inline const int& tagID() const {
      return _tag_id;
    }

    //! @brief const accessor
    inline const int& hammingDistance() const {
      return _hamming_distance;
    }

    //! @brief const accessor
    inline const float& decisionMargin() const {
      return _decision_margin;
    }

    //! @brief const accessor
    inline const srrg2_core::Matrix3f& homography() const {
      return _homography;
    }

    //! @brief const accessor
    inline const srrg2_core::Vector2f& center() const {
      return _center;
    }

    //! @brief const accessor
    inline const Vector2fVector& corners() const {
      return _corners;
    }

    //! @brief const accessor
    inline const srrg2_core::Vector2f& corner(const size_t& index_) const {
      assert(index_ < _corners.size() && "Apriltag_::corner|ERROR, index exceeds corners size");
      return _corners[index_];
    }

  public:
    //! @brief computes the pose of the tag in the camera frame (3d pose). it uses opencv PnP solver
    //!        and it assumes that the camera is a pinhole camera
    //! @param[out] tag_relative_pose_: computed pose tag in camera (camera frame is composed as: z
    //!                                 forward, x right, y down)
    //! @param[in] tag_size_: side length of black square in meters
    //! @param[in] camera_matrix_: camera matrix of the camera
    //! @param[in] distortion_coeffs: distortion coefficients (k_1, k_2, p_1, p_2); default
    //!                                    means that no distortion is taken into account
    //! @param[in] do_ransac_: use ransac to compute the pose
    void
    computeTagPose(srrg2_core::Isometry3f& tag_relative_pose_,
                   const float& tag_size_,
                   const srrg2_core::Matrix3f& camera_matrix_,
                   const srrg2_core::Vector4f& distortion_coeffs_ = srrg2_core::Vector4f::Zero(),
                   const bool do_ransac_                          = false) const;

    void draw(cv::Mat& cv_bgr_image_) const;

  protected:
    //! @brief the ID of the tag detected
    int _tag_id = -1;

    //! @brief How many error bits were corrected? Note: accepting large numbers of
    //! corrected errors leads to greatly increased false positive rates. In Apriltag3 library,
    //! maximum hamming distance is 2
    int _hamming_distance = -1;

    //! @brief A measure of the quality of the binary decoding process: the
    //! average difference between the intensity of a data bit versus
    //! the decision threshold. Higher numbers roughly indicate better
    //! decodes. This is a reasonable measure of detection accuracy
    //! only for very small tags-- not effective for larger tags
    float _decision_margin = -1.f;

    //! @brief A 3x3 homography that computes pixel coordinates from tag-relative coordinates. This
    //! homography describes the projection from an "ideal" tag (with corners at (-1,1), (1,1),
    //! (1,-1), and (-1,-1)) to pixels in the image.
    srrg2_core::Matrix3f _homography = srrg2_core::Matrix3f::Identity();

    //! @brief center of the detected tag (in fractional pixel coordinates)
    srrg2_core::Vector2f _center = srrg2_core::Vector2f::Zero();

    //! @brief the corners of the the detected tag - always counter clockwise  (in fractional pixel
    //! coordinates)
    Vector2fVector _corners;
  };

  //! @brief usings
  using Apriltag36h11 = Apriltag_<APRILTAG_FAMILY::Family_36h11>;

  //! @brief containers
  template <APRILTAG_FAMILY apriltag_family_>
  using ApriltagVector_ =
    std::vector<Apriltag_<apriltag_family_>, Eigen::aligned_allocator<Apriltag_<apriltag_family_>>>;
  template <APRILTAG_FAMILY apriltag_family_>
  using IDApriltagUMap_ = std::unordered_map<size_t,
                                             Apriltag_<apriltag_family_>,
                                             std::hash<size_t>,
                                             std::equal_to<size_t>,
                                             Eigen::aligned_allocator<Apriltag_<apriltag_family_>>>;
  template <APRILTAG_FAMILY apriltag_family_>
  using IDApriltagMap_ = std::map<size_t,
                                  Apriltag_<apriltag_family_>,
                                  std::less<size_t>,
                                  Eigen::aligned_allocator<Apriltag_<apriltag_family_>>>;

} /* namespace srrg2_apriltag_calibration */

#include "apriltag.hpp"
