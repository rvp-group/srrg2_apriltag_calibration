namespace srrg2_apriltag_calibration {

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  ApriltagCalibrator_<lidar_type_, apriltag_family_>::ApriltagCalibrator_() {
#ifndef NDEBUG
    std::cerr << "ApriltagCalibrator_::ApriltagCalibrator_|instanciating\n";
#endif
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  ApriltagCalibrator_<lidar_type_, apriltag_family_>::~ApriltagCalibrator_() {
#ifndef NDEBUG
    std::cerr << "ApriltagCalibrator_::~ApriltagCalibrator_|destroying\n";
#endif

    for (CalibrationObservation* obs : _calibration_observations_container) {
      if (obs) {
        delete obs;
      }
    }
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagCalibrator_<lidar_type_, apriltag_family_>::setMessage(
    const srrg2_core::BaseSensorMessagePtr& msg_) {
    _initial_guess = srrg2_core::geometry3d::v2t(param_inital_guess.value());

    // ia extract and cache messages
    _lidar_msg_ptr =
      extractMessage<srrg2_core::PointCloud2Message>(msg_, param_topic_lidar.value());
    _camera_msg_ptr = extractMessage<srrg2_core::ImageMessage>(msg_, param_topic_camera.value());
    _camera_info_msg_ptr =
      extractMessage<srrg2_core::CameraInfoMessage>(msg_, param_topic_camera_info.value());

    if (!(_lidar_msg_ptr && _camera_msg_ptr && _camera_info_msg_ptr)) {
      throw std::runtime_error("ApriltagCalibrator_::setMessage|ERROR, invalid message pack, "
                               "lidar, camera and camera info are required");
    }

    // ia clean stuff - cleaning always first you stupid dumbass
    _current_intensity_image_lidar.clear();
    _current_intensity_image_camera.clear();
    _common_detected_tag_id.clear();
    _current_calibration_observations.clear();

    // ia cache camera matrix
    _camera_matrix = _camera_info_msg_ptr->camera_matrix.value();

    // ia detect apriltags
    _detect();

    // ia reserve space for the new factor points
    assert(_fixed_points.size() == _moving_points.size() &&
           _moving_points.size() == _correspondences.size() &&
           "ApriltagCalibrator_::setMessage|ERROR, this should not happen");
    const size_t num_common_detections = _common_detected_tag_id.size();
    const size_t prev_corr_size        = _correspondences.size();
    const size_t new_reserved_size     = prev_corr_size + (num_common_detections * 5);
    _fixed_points.reserve(new_reserved_size);
    _moving_points.reserve(new_reserved_size);
    _correspondences.reserve(new_reserved_size);

    // ia cache last common detections entities to do visualization
    _current_calibration_observations.reserve(num_common_detections);

    // ia populate fixed and moving and correspondeces according to current detections
    for (const size_t& common_tag_id : _common_detected_tag_id) {
      const ApriltagType& camera_tag = _detector_camera.detections().at(common_tag_id);
      const ApriltagType& lidar_tag  = _detector_lidar.detections().at(common_tag_id);
      assert(camera_tag.tagID() == lidar_tag.tagID() &&
             camera_tag.corners().size() + 1 == lidar_tag.corners().size() + 1 &&
             "ApriltagCalibrator_::setMessage|ERROR, invalid detections");

      // ia TODO bleargh super shit
      CalibrationObservation* entry = new CalibrationObservation();
      _generateObservations(camera_tag, lidar_tag, entry);
      _calibration_observations_container.push_back(entry);
      _current_calibration_observations.emplace_back(entry);

      // ia cache fixed, moving and correspondences
      const size_t association_idx_offset = _correspondences.size();
      for (size_t i = 0; i < entry->valid_points; ++i) {
        const size_t association_index = association_idx_offset + i;
        _fixed_points.emplace_back(entry->camera_points[i]);
        _moving_points.emplace_back(entry->lidar_points_transf[i]);

        assert(association_index == _fixed_points.size() - 1 &&
               association_index == _moving_points.size() - 1 &&
               "ApriltagCalibrator_::setMessage|ERROR, invalid indexing");

        srrg2_core::Correspondence c(association_index, association_index);
        _correspondences.emplace_back(c);
      }
    }
#ifndef NDEBUG
    std::cerr << "ApriltagCalibrator_::setMessage|accumulated correspondences [ "
              << _correspondences.size() << "] \n";
#endif
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagCalibrator_<lidar_type_, apriltag_family_>::compute() {
    PROFILE_TIME("ApriltagCalibrator::compute");
    if (!_correspondences.size()) {
      std::cerr << "ApriltagCalibrator_::compute|WARINING, no correspondences, exit\n";
      return;
    }

#ifndef NDEBUG
    std::cerr << "ApriltagCalibrator_::compute|fixed size = " << _fixed_points.size() << std::endl;
    std::cerr << "ApriltagCalibrator_::compute|moving size = " << _moving_points.size()
              << std::endl;
    std::cerr << "ApriltagCalibrator_::compute|corr size = " << _correspondences.size()
              << std::endl;
#endif

    // ia now we have to populate the factor graph
    srrg2_solver::FactorGraphPtr factor_graph(new srrg2_solver::FactorGraph);

    // ia first the pose
    VariableTypePtr pose_vertex(new VariableType);
    pose_vertex->setStatus(srrg2_solver::VariableBase::Status::Active);
    pose_vertex->setGraphId(0);
    pose_vertex->setEstimate(srrg2_core::Isometry3f::Identity());
    factor_graph->addVariable(pose_vertex);

    // ia then we construct factors out of calibration data
    FactorTypePtr posit_factor(new FactorType);
    posit_factor->setGraphId(100);
    posit_factor->setVariableId(0, 0);
    posit_factor->setRobustifier(nullptr);
    posit_factor->setImageDim(srrg2_core::Vector2f(_current_intensity_image_camera.rows(),
                                                   _current_intensity_image_camera.cols()));
    posit_factor->setCameraMatrix(this->_camera_matrix);
    posit_factor->setFixed(_fixed_points);
    posit_factor->setMoving(_moving_points);
    posit_factor->setCorrespondences(_correspondences);
    posit_factor->setInformationMatrix(FactorType::InformationMatrixType::Identity());
    posit_factor->setEnabled(true);

    factor_graph->addFactor(posit_factor);
    factor_graph->bindFactors();
    assert(factor_graph->factors().size() == static_cast<size_t>(1) &&
           "ApriltagCalibrator_::compute|ERROR, factor size mismatch");

    param_solver->setGraph(factor_graph);
    param_solver->compute();
    const auto& stats = param_solver->iterationStats();
    std::cerr << "ApriltagCalibrator_::compute|perfomed [ " << stats.size() << " ] iterations\n"
              << stats;

    _estimate = pose_vertex->estimate() * _initial_guess;
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void
  ApriltagCalibrator_<lidar_type_, apriltag_family_>::dump(const std::string& filename_) const {
    if (!_calibration_observations_container.size()) {
      std::cerr << "ApriltagCalibrator_::dump|WARINING, no common detections between camera and "
                   "lidar, nothing to save"
                << std::endl;
      return;
    }

    std::ofstream output_filestream(filename_);
    output_filestream << "## <tag_id> <4 tag corners in image plane (u,v)> <tag center in image "
                         "plane (u,v)> <4 tag "
                         "corners in laser frame (x,y,z)> <tag center in laser frame (x,y,z)>\n";
    for (auto entry : _calibration_observations_container) {
      output_filestream << *entry << std::endl;
    }

    output_filestream.close();
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void
  ApriltagCalibrator_<lidar_type_, apriltag_family_>::save(const std::string& filename_) const {
    std::ofstream output_filestream(filename_);
    output_filestream << "## final estimate [x y z qx qy qz]\n";
    output_filestream << "ESTIMATE ";
    const srrg2_core::Vector6f estimate_vec = srrg2_core::geometry3d::t2tnq(_estimate);
    for (int i = 0; i < estimate_vec.rows(); ++i) {
      output_filestream << std::fixed << std::setprecision(5) << estimate_vec[i] << " ";
    }

    output_filestream.close();
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagCalibrator_<lidar_type_, apriltag_family_>::draw(
    const srrg2_core::ViewerCanvasPtr& canvas_) const {
    // ia draw reference frame of the lidar
    canvas_->putReferenceSystem(2.f);

    // ia draw the current transformed lidar cloud TODO DIOCANE
    srrg2_core::Point3fVectorCloud point_cloud_transf;
    point_cloud_transf.resize(_current_lidar_cloud.size());
    _current_lidar_cloud.copyFieldTo<0, 0, srrg2_core::Point3fVectorCloud>(point_cloud_transf);
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fCyan());
    canvas_->pushPointSize();
    canvas_->setPointSize(1.f);
    canvas_->putPoints(point_cloud_transf);
    canvas_->popAttribute();
    canvas_->popAttribute();

    // ia draw the current detections in lidar
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fViolet());
    canvas_->pushPointSize();
    canvas_->setPointSize(5.f);
    for (size_t k = 0; k < _current_calibration_observations.size(); ++k) {
      canvas_->putPoints(_current_calibration_observations[k]->lidar_points);
    }
    canvas_->popAttribute();
    canvas_->popAttribute();

    cv::Mat cv_canvas;
    drawProjectiveAssociations(cv_canvas);
    canvas_->putImage(cv_canvas);
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagCalibrator_<lidar_type_, apriltag_family_>::_detect() {
    PROFILE_TIME("ApriltagCalibrator::preprocessing-and-detection");
    assert(_lidar_msg_ptr && "ApriltagCalibrator_::_detect|ERROR, no laser msg");
    assert(_camera_msg_ptr && "ApriltagCalibrator_::_detect|ERROR, no camera msg");

    // ia preprocess lidar data
    _preprocessLidarMsg();
    assert(_current_intensity_image_lidar.rows() && _current_intensity_image_lidar.cols() &&
           "ApriltagCalibrator_::_detect|ERROR, invalid lidar intensity image");

    // ia preprocess camera data
    _preprocessCameraMsg();
    assert(_current_intensity_image_camera.rows() && _current_intensity_image_camera.cols() &&
           "ApriltagCalibrator_::_detect|ERROR, invalid camera intensity image");

    // ia fire the engine
    _detector_lidar.setImage(&_current_intensity_image_lidar);
    _detector_lidar.compute();

    // ia fire the engine
    _detector_camera.setImage(&_current_intensity_image_camera);
    _detector_camera.compute();

    // ia if we do not have both detections return
    if (!_detector_lidar.numDetections() || !_detector_camera.numDetections()) {
      return;
    }

    // ia check whether we have common detections
    for (const auto& d : _detector_lidar.detections()) {
      if (!_detector_camera.detections().count(d.first)) {
        continue;
      }
      _common_detected_tag_id.insert(d.first);
    }

#ifndef NDEBUG
    std::cerr << "ApriltagCalibrator_::_detect|detected [ " << _common_detected_tag_id.size()
              << " ] common tags\n";
#endif
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagCalibrator_<lidar_type_, apriltag_family_>::_preprocessLidarMsg() {
    // ia project the pointcloud
    srrg2_core::ImageFloat lidar_raw_intensity_image;
    _projectLidarCloud(lidar_raw_intensity_image);

    // ia normalize the image
    const float scaling_factor = 1.f / param_max_intensity.value();
    lidar_raw_intensity_image.scale(scaling_factor);

    // ia convert the image to uchar using opencv because this shit is sick
    //    lidar_raw_intensity_image.convertTo(_current_intensity_image_lidar, 255.f);
    cv::Mat cv_raw_intensity, cv_uchar_intensity;
    lidar_raw_intensity_image.toCv(cv_raw_intensity);
    cv_raw_intensity.convertTo(cv_uchar_intensity, CV_8UC1, 255.0);

    assert(!cv_uchar_intensity.empty() && "ApriltagCalibrator_::_preprocessLidarMsg|ERROR, invalid "
                                          "conversion to cv UCHAR image");

    _current_intensity_image_lidar.fromCv(cv_uchar_intensity);
    assert(_current_intensity_image_lidar.rows() == lidar_raw_intensity_image.rows() &&
           _current_intensity_image_lidar.cols() == lidar_raw_intensity_image.cols() &&
           "ApriltagCalibrator_::_preprocessLidarMsg|ERROR, invalid conversion to UCHAR image");
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagCalibrator_<lidar_type_, apriltag_family_>::_preprocessCameraMsg() {
    switch (_camera_msg_ptr->image()->type()) {
      case srrg2_core::ImageType::TYPE_8UC3: {
        // ia original image is rgb so we need to convert it to bgr to do things right
        // ia TODO bleagh
        cv::Mat cv_rgb, cv_grayscale;
        _camera_msg_ptr->image()->toCv(cv_rgb);
        cv::cvtColor(cv_rgb, cv_grayscale, CV_RGB2GRAY);
        _current_intensity_image_camera.fromCv(cv_grayscale);
        break;
      }
      case srrg2_core::ImageType::TYPE_8UC1: {
        // ia deep copy the shit TODO bleargh
        _current_intensity_image_camera =
          *(dynamic_cast<srrg2_core::ImageUInt8*>(_camera_msg_ptr->image()));
        break;
      }
      default:
        throw std::runtime_error(
          "ApriltagCalibrator_::_preprocessCameraMsg|ERROR, invalid image type [ " +
          std::to_string(_camera_msg_ptr->image()->type()) + " ]");
    }

    assert(_current_intensity_image_camera.rows() == _camera_msg_ptr->image()->rows() &&
           _current_intensity_image_camera.cols() == _camera_msg_ptr->image()->cols() &&
           "ApriltagCalibrator_::_preprocessCameraMsg|ERROR, invalid conversion of camera message");
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagCalibrator_<lidar_type_, apriltag_family_>::_projectLidarCloud(
    srrg2_core::ImageFloat& lidar_raw_intensity_) {
    PROFILE_TIME("ApriltagCalibrator::lidar-projection");
    assert(param_lidar_projector.value() &&
           "ApriltagCalibrator_::_getIntensityImageFromLidar|ERROR, no projector set");

    // ia get the point cloud
    _current_lidar_cloud_projected.clear();
    _current_lidar_cloud.clear();
    _lidar_msg_ptr->getPointCloud(_current_lidar_cloud);

    // ia project the cloud
    param_lidar_projector->compute(
      _current_lidar_cloud_projected, _current_lidar_cloud.begin(), _current_lidar_cloud.end());

    // ia how to get constant access to a fuckin property (everytime that I access something I
    // actually touch the variable so I will do the reinitialization evertyime)
    const size_t& projection_num_columns =
      std::const_pointer_cast<const LidarProjectorType>(param_lidar_projector.value())
        ->param_num_columns.value();

    // ia get the intensity image
    lidar_raw_intensity_.resize(_lidar_sensor.verticalResolution(), projection_num_columns);
    auto intensity_image_it = lidar_raw_intensity_.begin();
    for (auto it = _current_lidar_cloud_projected.begin();
         it != _current_lidar_cloud_projected.end();
         ++it, ++intensity_image_it) {
      *intensity_image_it = it->source_it->intensity();
    }
  }

  //! @brief fills one apriltag calibration entry
  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagCalibrator_<lidar_type_, apriltag_family_>::_generateObservations(
    const ApriltagType& camera_tag_,
    const ApriltagType& lidar_tag_,
    CalibrationObservation* obs_) {
    // ia allocate space
    obs_->camera_points.reserve(camera_tag_.corners().size() + 1);
    obs_->lidar_points.reserve(lidar_tag_.corners().size() + 1);
    obs_->lidar_points_transf.reserve(lidar_tag_.corners().size() + 1);
    obs_->lidar_points_transf_proj.reserve(lidar_tag_.corners().size() + 1);

    // ia jic
    obs_->tag_id = camera_tag_.tagID();

    // ia copy laser corners (x,y,z) and cache (u,v) projections
    srrg2_core::Point3f p_lidar;
    srrg2_core::Point3f p_lidar_transformed;
    srrg2_core::Point2f pp_lidar;
    for (const auto& corner : lidar_tag_.corners()) {
      const srrg2_core::Vector3f& lidar_coords =
        _current_lidar_cloud_projected.at(corner.y(), corner.x()).source_it->coordinates();
      p_lidar.coordinates() = lidar_coords;
      obs_->lidar_points.emplace_back(p_lidar);

      p_lidar_transformed.coordinates() = _initial_guess * lidar_coords;
      obs_->lidar_points_transf.emplace_back(p_lidar_transformed);

      pp_lidar.coordinates() = _pinholeProject(_camera_matrix,
                                               srrg2_core::Isometry3f::Identity(),
                                               p_lidar_transformed.coordinates(),
                                               _camera_msg_ptr->image_cols.value(),
                                               _camera_msg_ptr->image_rows.value());
      obs_->lidar_points_transf_proj.emplace_back(pp_lidar);
    }

    // ia copy laser center (x y z)
    const srrg2_core::Vector3f& lidar_coords =
      _current_lidar_cloud_projected.at(lidar_tag_.center().y(), lidar_tag_.center().x())
        .source_it->coordinates();
    p_lidar.coordinates()             = lidar_coords;
    obs_->lidar_points.emplace_back(p_lidar);
    
    p_lidar_transformed.coordinates() = _initial_guess * lidar_coords;
    obs_->lidar_points_transf.emplace_back(p_lidar_transformed);

    pp_lidar.coordinates() = _pinholeProject(_camera_matrix,
                                             srrg2_core::Isometry3f::Identity(),
                                             p_lidar_transformed.coordinates(),
                                             _camera_msg_ptr->image_cols.value(),
                                             _camera_msg_ptr->image_rows.value());
    obs_->lidar_points_transf_proj.emplace_back(pp_lidar);

    // ia this is now hardcoded, but maybe you can discard corners of the lidar tag detection that
    // are not coplanar w.r.t. the others, therefore you will have a smaller amount of valid points
    obs_->valid_points = 5;

    // ia copy camera corners (u.v)
    srrg2_core::Point2f p_camera;
    for (const auto& corner : camera_tag_.corners()) {
      p_camera.coordinates() << corner.x(), corner.y();
      obs_->camera_points.emplace_back(p_camera);
    }

    // ia copy camera center (u,v)
    p_camera.coordinates() << camera_tag_.center().x(), camera_tag_.center().y();
    obs_->camera_points.emplace_back(p_camera);
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  void ApriltagCalibrator_<lidar_type_, apriltag_family_>::drawProjectiveAssociations(
    cv::Mat& cv_canvas_) const {
    // ia projections are all in the camera frame, so we take the original message and convert it to
    // a nice and showable rgb image
    cv::Mat cv_gray;
    _current_intensity_image_camera.toCv(cv_gray);
    cv::cvtColor(cv_gray, cv_canvas_, CV_GRAY2BGR);

    for (CalibrationObservation* entry_ptr : _current_calibration_observations) {
      assert(entry_ptr && "ApriltagCalibrator_::drawProjectiveAssociations|ERROR, invalid entry");
      assert(entry_ptr->camera_points.size() == entry_ptr->lidar_points_transf.size() &&
             "ApriltagCalibrator_::drawProjectiveAssociations|ERROR, entry points mismatch");
      const size_t& num_points = entry_ptr->camera_points.size();
      for (size_t k = 0; k < num_points; ++k) {
        // ia construct cv points
        cv::Point2f cv_p_cam(entry_ptr->camera_points[k].coordinates().x(),
                             entry_ptr->camera_points[k].coordinates().y());

        cv::Point2f cv_p_lidar(entry_ptr->lidar_points_transf_proj[k].coordinates().x(),
                               entry_ptr->lidar_points_transf_proj[k].coordinates().y());

        // ia draw
        cv::circle(cv_canvas_, cv_p_cam, 2, cv::Scalar(0, 0, 255, 0), 2);
        cv::circle(cv_canvas_, cv_p_lidar, 2, cv::Scalar(255, 0, 0, 0), 2);
        cv::line(cv_canvas_, cv_p_cam, cv_p_lidar, cv::Scalar(0, 255, 0, 0), 1);
      }
    }
  }

  template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_,
            srrg2_apriltag_calibration::APRILTAG_FAMILY apriltag_family_>
  srrg2_core::Vector2f ApriltagCalibrator_<lidar_type_, apriltag_family_>::_pinholeProject(
    const srrg2_core::Matrix3f& camera_,
    const srrg2_core::Isometry3f& transf,
    const srrg2_core::Vector3f& p,
    const size_t& cols_,
    const size_t& rows_) const {
    assert(rows_ > 0 && cols_ > 0 &&
           "ApriltagCalibrator_::_pinholeProject|ERROR, invalid image size");
    srrg2_core::Vector2f pp = srrg2_core::Vector2f::Zero();
    pp << -1.f, -1.f;

    const srrg2_core::Vector3f p_transf = transf * p;
    if (p_transf.z() < 0) {
      return pp;
    }

    const float iz                    = 1. / p_transf.z();
    const srrg2_core::Vector3f cam_pt = camera_ * p_transf;
    const srrg2_core::Vector2f img_pt(cam_pt.x() * iz, cam_pt.y() * iz);

    if (img_pt.x() < 0 || img_pt.x() > cols_) {
      return pp;
    }
    if (img_pt.y() < 0 || img_pt.y() > rows_) {
      return pp;
    }

    pp = img_pt;
    return pp;
  }

} /* namespace srrg2_apriltag_calibration */
