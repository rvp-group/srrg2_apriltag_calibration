namespace srrg2_apriltag_calibration {
  template <srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  ApriltagDetector_<apriltag_family_>::ApriltagDetector_() {
    switch (ApriltagFamily) {
      case APRILTAG_FAMILY::Family_36h11:
#ifndef NDEBUG
        std::cerr << "ApriltagDetector_::ApriltagDetector_|constructing family [ 36h11 ]\n";
#endif
        _tag_family = tag36h11_create();
        break;
      default:
        throw std::runtime_error(
          "ApriltagDetector_::ApriltagDetector_|ERROR, invalid apriltag family");
    }
    _tag_detector = apriltag_detector_create();
    apriltag_detector_add_family(_tag_detector, _tag_family);
  }

  template <srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  ApriltagDetector_<apriltag_family_>::~ApriltagDetector_() {
#ifndef NDEBUG
    std::cerr << "ApriltagDetector_::ApriltagDetector_|destroying\n";
#endif
    if (_tag_family) {
      switch (ApriltagFamily) {
        case APRILTAG_FAMILY::Family_36h11:
          tag36h11_destroy(_tag_family);
          break;
        default:
          std::cerr << "ApriltagDetector_::ApriltagDetector_|ERROR, invalid apriltag family, "
                       "cannot destroy the object. MEMORY LEAK is happening\n";
      }
    }

    if (_tag_detector) {
      apriltag_detector_destroy(_tag_detector);
    }
  }

  template <srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  size_t ApriltagDetector_<apriltag_family_>::_detect(const cv::Mat& uchar_image_,
                                                      ApriltagContainerType& detections_) {
    assert(!uchar_image_.empty() && "ApriltagDetector_::_detect|ERROR, empty image");
    assert(uchar_image_.type() == CV_8UC1 && "ApriltagDetector_::_detect|ERROR, invalid image");
    assert(_tag_detector && "ApriltagDetector_::_detect|ERROR, no tag detector");

    // ia just in case
    size_t num_detections = 0;
    if (detections_.size()) {
      detections_.clear();
    }

    // ia convert into library format
    image_u8 at_image = {.width  = uchar_image_.cols,
                         .height = uchar_image_.rows,
                         .stride = uchar_image_.cols,
                         .buf    = uchar_image_.data};

    // ia do the magic trick
    zarray* at_detections = apriltag_detector_detect(_tag_detector, &at_image);
    num_detections        = zarray_size(at_detections);

    if (!num_detections) {
#ifndef NDEBUG
      std::cerr << "ApriltagDetector_::_detect|WARINING, no detections\n";
#endif
      apriltag_detections_destroy(at_detections);
      return num_detections;
    }

    // ia populate our detection structure
    for (size_t i = 0; i < num_detections; ++i) {
      apriltag_detection* at_current_detection = NULL;
      zarray_get(at_detections, i, &at_current_detection);

      ApriltagType detected_tag(at_current_detection);
      detections_.insert(std::make_pair(detected_tag.tagID(), detected_tag));
    }

    // ia checkout
    apriltag_detections_destroy(at_detections);

    return num_detections;
  }

  template <srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagDetector_<apriltag_family_>::compute() {
    assert(_intensity_image && "ApriltagDetector_::compute|ERROR, invalid image");
    this->_intensity_image->toCv(_cv_intensity_image);

    assert(this->_intensity_image->rows() == (size_t) _cv_intensity_image.rows &&
           this->_intensity_image->cols() == (size_t) _cv_intensity_image.cols &&
           "ApriltagDetector_::compute|ERROR, invalid conversion");

    // ia convert also to rgb
    cv::cvtColor(_cv_intensity_image, _cv_color_image, CV_GRAY2BGR);

    assert(this->_intensity_image->rows() == (size_t) _cv_color_image.rows &&
           this->_intensity_image->cols() == (size_t) _cv_color_image.cols &&
           "ApriltagDetector_::compute|ERROR, invalid color conversion");

    // ia start detection
    _num_detections = 0;
    _detected_tags.clear();
    _detect(_cv_intensity_image, _detected_tags);
    _num_detections = _detected_tags.size();

#ifndef NDEBUG
    std::cerr << "ApriltagDetector_::compute|detected [ " << _num_detections << " ] tags "
              << std::endl;
#endif
  }

  template <srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagDetector_<apriltag_family_>::draw(cv::Mat& cv_canvas_) const {
    assert(!_cv_color_image.empty() && "ApriltagDetector_::draw|ERROR, empty color image");
    _cv_color_image.copyTo(cv_canvas_);
    for (const auto& d : _detected_tags) {
      d.second.draw(cv_canvas_);
    }
  }

} // namespace srrg2_apriltag_calibration
