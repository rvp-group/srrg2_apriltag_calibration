#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_test/test_helper.hpp>

#include "fixtures.hpp"
#include "srrg_apriltag_detection/instances.h"

using namespace srrg2_core;
using namespace srrg2_apriltag_calibration;

int main(int argc, char** argv) {
  return srrg2_test::runTests(argc, argv, true /*use test folder*/);
}

TEST_F(Ouster, ApriltagDetector36h11) {
  srrg2_apriltagDetector_registerTypes();

  ApriltagDetector36h11Ptr detector(new ApriltagDetector36h11);
  assert(detector && "cannot create detector");
  detector->setImage(&_intensity_image);
  detector->compute();
  ASSERT_EQ(detector->numDetections(), static_cast<size_t>(2));

  //  detector->draw();
  //  cv::waitKey(0);
}

TEST_F(Flir, ApriltagDetector36h11) {
  srrg2_apriltagDetector_registerTypes();

  ApriltagDetector36h11Ptr detector(new ApriltagDetector36h11);
  assert(detector && "cannot create detector");
  detector->setImage(&_intensity_image);
  detector->compute();
  ASSERT_EQ(detector->numDetections(), static_cast<size_t>(3));
  //  detector->draw();
  //  cv::waitKey(0);
}
