#include <srrg_boss/blob.h>

#include "instances.h"

using namespace srrg2_core;

namespace srrg2_apriltag_calibration {
  void srrg2_apriltagDetector_registerTypes() {
    BOSS_REGISTER_CLASS(ApriltagDetector36h11);
  }
} // namespace srrg2_apriltag_calibration
