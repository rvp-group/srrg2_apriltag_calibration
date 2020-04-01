#pragma once
#include <srrg_messages/instances.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_system_utils/shell_colors.h>

#include <srrg_test/test_helper.hpp>

#include "srrg_apriltag_utils/utilities.h"

using namespace srrg2_core;

//! @brief class that extends gtest and loads some ICL images
//! when you derive from GTest class, you must implement the SetUp and TearDown methods
class Ouster : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  //! @brief called right before executing the test
  //! loads everything and so on
  void SetUp() override {
    std::cerr << "Ouster::SetUp|test data directory: " << FG_YELLOW(_base_data_path) << std::endl;
    // ia load image
    std::string image_filename = _base_data_path + "/ouster_intensity_0.png";

    if (!srrg2_apriltag_calibration::checkFile(image_filename)) {
      throw std::runtime_error("Ouster::SetUp|ERROR, cannot find file [ " + image_filename + " ]");
    }
    _cv_intensity_image = cv::imread(image_filename, CV_LOAD_IMAGE_GRAYSCALE);
    ASSERT_EQ((size_t) _cv_intensity_image.rows, (size_t) 64);
    ASSERT_EQ((size_t) _cv_intensity_image.cols, (size_t) 1024);
    _intensity_image.fromCv(_cv_intensity_image);

    ASSERT_EQ((size_t) _cv_intensity_image.rows, _intensity_image.rows());
    ASSERT_EQ((size_t) _cv_intensity_image.cols, _intensity_image.cols());
  }

  //! @brief called right before dtor
  //! destroys things
  void TearDown() override {
  }

protected:
  srrg2_core::ImageUInt8 _intensity_image;
  cv::Mat _cv_intensity_image;

private:
  //! @brief super specific things
  const std::string _base_data_path = SRRG2_APRILTAG_CALIBRATION_TEST_DATA_FOLDER;
};

//! @brief class that extends gtest and loads some ICL images
//! when you derive from GTest class, you must implement the SetUp and TearDown methods
class Flir : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  //! @brief called right before executing the test
  //! loads everything and so on
  void SetUp() override {
    std::cerr << "Flir::SetUp|test data directory: " << FG_YELLOW(_base_data_path) << std::endl;
    // ia load image
    std::string image_filename = _base_data_path + "/flir_mono_0.png";

    if (!srrg2_apriltag_calibration::checkFile(image_filename)) {
      throw std::runtime_error("Flir::SetUp|ERROR, cannot find file [ " + image_filename + " ]");
    }

    _cv_intensity_image = cv::imread(image_filename, CV_LOAD_IMAGE_GRAYSCALE);
    ASSERT_EQ((size_t) _cv_intensity_image.rows, (size_t) 512);
    ASSERT_EQ((size_t) _cv_intensity_image.cols, (size_t) 1024);
    _intensity_image.fromCv(_cv_intensity_image);

    ASSERT_EQ((size_t) _cv_intensity_image.rows, _intensity_image.rows());
    ASSERT_EQ((size_t) _cv_intensity_image.cols, _intensity_image.cols());
  }

  //! @brief called right before dtor
  //! destroys things
  void TearDown() override {
  }

protected:
  srrg2_core::ImageUInt8 _intensity_image;
  cv::Mat _cv_intensity_image;

private:
  //! @brief super specific things
  const std::string _base_data_path = SRRG2_APRILTAG_CALIBRATION_TEST_DATA_FOLDER;
};
