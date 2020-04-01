#pragma once

#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_image/image.h>

#include "apriltag.h"
#include "srrg_apriltag_utils/utilities.h"

namespace srrg2_apriltag_calibration {

  //! @brief base class, with couple of default parameters and the APIs
  class ApriltagDetectorBase : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ia object life
    ApriltagDetectorBase() {
    }

    // ia default object life
    virtual ~ApriltagDetectorBase() = default;

  public:
    virtual void setImage(srrg2_core::ImageUInt8* intensity_image_) {
      assert(intensity_image_ && "ApriltagDetectorBase::setImage|ERROR, invalid image");
      _intensity_image = intensity_image_;
    }

    //! @brief does the job
    virtual void compute() = 0;

    //! @brief shows detections using opencv
    virtual void draw(cv::Mat& cv_canvas_) const = 0;

    //! @brief inline accessor to the number of detections
    inline const size_t& numDetections() const {
      return _num_detections;
    }

  protected:
    //! @brief intensity image that we will use to detect tag (grayscale)
    srrg2_core::ImageUInt8* _intensity_image = nullptr;

    //! @brief opencv images
    cv::Mat _cv_color_image;
    cv::Mat _cv_intensity_image;

    //! @brief statistics
    size_t _num_detections = 0;
  };

  using ApriltagDetectorBasePtr = std::shared_ptr<ApriltagDetectorBase>;

  template <srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  class ApriltagDetector_ : public ApriltagDetectorBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ia usings
    static constexpr APRILTAG_FAMILY ApriltagFamily = apriltag_family_;
    using ApriltagType                              = Apriltag_<ApriltagFamily>;
    using ApriltagContainerType                     = IDApriltagUMap_<ApriltagFamily>;
    using TagIDSet                                  = std::set<size_t>;

  public:
    //! @brief object life
    ApriltagDetector_();
    virtual ~ApriltagDetector_();

    //! @brief override
    void compute() override;

    //! @brief override
    void draw(cv::Mat& cv_canvas_) const override;

    //! @brief inline getter
    inline const ApriltagContainerType& detections() const {
      return _detected_tags;
    }

  protected:
    //! @brief detection container
    ApriltagContainerType _detected_tags;

    //! @brief apriltag3 things
    apriltag_family* _tag_family     = nullptr;
    apriltag_detector* _tag_detector = nullptr;

  protected:
    //! @brief auxiliary functions
    size_t _detect(const cv::Mat& uchar_image_, ApriltagContainerType& detections_);
  };

  using ApriltagDetector36h11    = ApriltagDetector_<APRILTAG_FAMILY::Family_36h11>;
  using ApriltagDetector36h11Ptr = std::shared_ptr<ApriltagDetector36h11>;

} // namespace srrg2_apriltag_calibration

#include "apriltag_detector.hpp"
