#pragma once
#include <iomanip>
#include <srrg_system_utils/profiler.h>

#include <srrg_image/image.h>
#include <srrg_messages/instances.h>
#include <srrg_pcl/camera_matrix_owner.h>
#include <srrg_pcl/point_projector_lidar3d_types.h>
#include <srrg_pcl/point_unprojector_lidar3d_types.h>

#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>
#include <srrg_solver/variables_and_factors/types_projective/se3_projective_error_factor.h>

#include <srrg_viewer/viewer_canvas.h>

#include "srrg_apriltag_detection/apriltag_detector.h"

namespace srrg2_apriltag_calibration {

  //! @brief TODO this is a shit that contains associations between camera and lidar points (in
  //! their frame)
  struct CalibrationObservation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int tag_id          = -1;
    size_t valid_points = 0;
    srrg2_core::Point3fVectorCloud lidar_points;             // ia original lidar points (x,y,z)
    srrg2_core::Point3fVectorCloud lidar_points_transf;      // ia lidar points in the camera frame
    srrg2_core::Point2fVectorCloud lidar_points_transf_proj; // ia projected lidar points (u,v)
    srrg2_core::Point2fVectorCloud camera_points;            // ia camera points (u,v)
  };
  using CalibrationObservationVector =
    std::vector<CalibrationObservation, Eigen::aligned_allocator<CalibrationObservation>>;
  using CalibrationObservationPtrVector = std::vector<CalibrationObservation*>;

  //! @brief apriltag calibration entry print overload
  std::ostream& operator<<(std::ostream& stream_, const CalibrationObservation& entry_) {
    stream_ << entry_.tag_id << " ";
    for (const auto& p : entry_.camera_points) {
      stream_ << p.coordinates().x() << " " << p.coordinates().y() << " ";
    }
    for (const auto& p : entry_.lidar_points) {
      stream_ << p.coordinates().x() << " " << p.coordinates().y() << " " << p.coordinates().z()
              << " ";
    }
    return stream_;
  }

  //! @brief base class for the extrinsic calibration of a 3d-lidar and a camera
  //!        the calibration works should work like this: you detect tags in both sensors, then you
  //!        minimize the reprojection error between 3dlidar points and 2d camera points. camera
  //!        matrix of the camera is required
  class ApriltagCalibratorBase : public srrg2_core::Configurable {
  public:
    using ThisType     = ApriltagCalibratorBase;
    using BaseType     = srrg2_core::Configurable;
    using VariableType = srrg2_solver::VariableSE3QuaternionRight;
    using FactorType   = srrg2_solver::SE3ProjectiveErrorFactorCorrespondenceDriven;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM(srrg2_core::PropertyEigen_<srrg2_core::Vector6f>,
          inital_guess,
          "initial transformation of T_lidar_IN_camera [x y z qx qy qz]",
          srrg2_core::Vector6f::Zero(),
          nullptr);

    ApriltagCalibratorBase() {
    }
    virtual ~ApriltagCalibratorBase() = default;

    //! @brief buffers the message, extracts intensity images and performs detection
    virtual void setMessage(const srrg2_core::BaseSensorMessagePtr& message_ptr_) = 0;

    //! @brief given data, calls the solver and shits out a transform
    virtual void compute() = 0;

    //! @brief saves extrinsic parameters
    virtual void save(const std::string& filename_) const = 0;

    //! @brief dumps calibration data into a txt file
    virtual void dump(const std::string& filename_) const = 0;

    //! @brief draws on canvas
    virtual void draw(const srrg2_core::ViewerCanvasPtr& canvas_) const = 0;

    //! @brief accessor to the estimate (only constant)
    inline const srrg2_core::Isometry3f& estimate() const {
      return _estimate;
    }

  protected:
    //! @brief caching the messages
    srrg2_core::PointCloud2MessagePtr _lidar_msg_ptr      = nullptr;
    srrg2_core::ImageMessagePtr _camera_msg_ptr           = nullptr;
    srrg2_core::CameraInfoMessagePtr _camera_info_msg_ptr = nullptr;

    //! @brief intensity images
    srrg2_core::ImageUInt8 _current_intensity_image_lidar;
    srrg2_core::ImageUInt8 _current_intensity_image_camera;

    //! @brief camera matrix
    srrg2_core::Matrix3f _camera_matrix = srrg2_core::Matrix3f::Zero();

    //! @brief raw lidar pcl
    srrg2_core::PointIntensity3fVectorCloud _current_lidar_cloud;

    //! @brief initial guess of the transform T_lidar_IN_camera
    srrg2_core::Isometry3f _initial_guess = srrg2_core::Isometry3f::Identity();

    //! @brief final estimate of the transform T_lidar_IN_camera
    srrg2_core::Isometry3f _estimate = srrg2_core::Isometry3f::Identity();

    //! @brief posit entries
    srrg2_core::Point2fVectorCloud _fixed_points;
    srrg2_core::Point3fVectorCloud _moving_points;
    srrg2_core::CorrespondenceVector _correspondences;

    //! @brief required to dump calibration data (ia for Cyrill bachelor student)
    CalibrationObservationPtrVector _calibration_observations_container;
    //! @brief view of the last frame, just for easy visualization
    CalibrationObservationPtrVector _current_calibration_observations;
  };

  using ApriltagCalibratorBasePtr = std::shared_ptr<ApriltagCalibratorBase>;

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  class ApriltagCalibrator_ : public ApriltagCalibratorBase, public srrg2_core::Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief base usings
    using BaseType        = ApriltagCalibratorBase;
    using ThisType        = ApriltagCalibrator_<lidar_type_, apriltag_family_>;
    using VariableType    = typename BaseType::VariableType;
    using VariableTypePtr = std::shared_ptr<VariableType>;
    using FactorType      = typename BaseType::FactorType;
    using FactorTypePtr   = std::shared_ptr<FactorType>;

    //! @breif actual usings
    static constexpr srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE LidarType = lidar_type_;
    static constexpr APRILTAG_FAMILY ApriltagFamily                        = apriltag_family_;
    using ApriltagDetectorType    = ApriltagDetector_<ApriltagFamily>;
    using ApriltagDetectorTypePtr = std::shared_ptr<ApriltagDetectorType>;
    using LidarSensorType         = srrg2_core::srrg2_lidar3d_utils::Lidar3DSensor_<LidarType>;
    using LidarProjectorType =
      srrg2_core::PointProjectorLidar3D_<LidarType, srrg2_core::PointIntensity3fVectorCloud>;
    using LidarUnprojectorType =
      srrg2_core::PointUnprojectorLidar3D_<LidarType, srrg2_core::PointIntensity3fVectorCloud>;
    using LidarProjectionMatrixType = typename LidarProjectorType::TargetMatrixType;
    using ApriltagType              = Apriltag_<ApriltagFamily>;
    using ApriltagContainerType     = IDApriltagUMap_<ApriltagFamily>;
    using TagIDSet                  = std::set<size_t>;

    //! @brief object life
    ApriltagCalibrator_();
    virtual ~ApriltagCalibrator_();

    //! @brief params
    PARAM(srrg2_core::PropertyConfigurable_<srrg2_solver::Solver>,
          solver,
          "LS solver as usual",
          srrg2_solver::SolverPtr(new srrg2_solver::Solver),
          0);
    PARAM(srrg2_core::PropertyConfigurable_<LidarProjectorType>,
          lidar_projector,
          "lidar projector",
          std::shared_ptr<LidarProjectorType>(new LidarProjectorType),
          0);
    PARAM(srrg2_core::PropertyFloat,
          max_intensity,
          "lidar intensity image normalization value",
          200,
          0);
    PARAM(srrg2_core::PropertyString,
          topic_lidar,
          "lidar point cloud topic",
          "/os1_cloud_node/points",
          0);
    PARAM(srrg2_core::PropertyString,
          topic_camera,
          "camera [rectified] images topic",
          "/camera/image_rect",
          0);
    PARAM(srrg2_core::PropertyString,
          topic_camera_info,
          "camera info topic",
          "/camera/camera_info",
          0);

    //! @brief override
    void setMessage(const srrg2_core::BaseSensorMessagePtr& message_ptr_) override;

    //! @brief override
    void compute() override;

    //! @brief override
    void save(const std::string& filename_) const override;

    //! @brief override
    void dump(const std::string& filename_) const override;

    //! @brief override
    void draw(const srrg2_core::ViewerCanvasPtr& canvas_) const override;

    //! @brief draw the raw detections on current message pack
    void drawRawDetections(cv::Mat& cv_canvas_camera_, cv::Mat& cv_canvas_lidar_) const {
      _detector_camera.draw(cv_canvas_camera_);
      _detector_lidar.draw(cv_canvas_lidar_);
    }

    //! @brief draw projective associations using opencv
    void drawProjectiveAssociations(cv::Mat& cv_canvas_) const;

  protected:
    //! @brief two detectors (to keep the things easy)
    ApriltagDetectorType _detector_lidar;
    ApriltagDetectorType _detector_camera;

    //! @brief lidar sensor
    LidarSensorType _lidar_sensor;

    //! @brief ids of the tags detected both by the camera and the lidar
    TagIDSet _common_detected_tag_id;

    //! @brief projeted lidar pcl
    LidarProjectionMatrixType _current_lidar_cloud_projected;

  protected:
    void _detect();
    void _preprocessLidarMsg();
    void _preprocessCameraMsg();
    void _projectLidarCloud(srrg2_core::ImageFloat& lidar_raw_intensity_);

    void _generateObservations(const ApriltagType& camera_tag_,
                               const ApriltagType& lidar_tag_,
                               CalibrationObservation* obs_);

    //! @brief base pinhole projection of a 3D point
    srrg2_core::Vector2f _pinholeProject(const srrg2_core::Matrix3f& camera_,
                                         const srrg2_core::Isometry3f& transf,
                                         const srrg2_core::Vector3f& p,
                                         const size_t& cols_,
                                         const size_t& rows_) const;
  };

  //! @brief family specialization
  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_>
  using ApriltagCalibrator36h11_ = ApriltagCalibrator_<lidar_type_, APRILTAG_FAMILY::Family_36h11>;

  //! @brief complete specialization
  using ApriltagCalibrator36h11_OS164 =
    ApriltagCalibrator36h11_<srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64>;
  using ApriltagCalibrator36h11_OS164Ptr = std::shared_ptr<ApriltagCalibrator36h11_OS164>;

} /* namespace srrg2_apriltag_calibration */

#include "apriltag_calibrator.hpp"
