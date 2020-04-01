#include "instances.h"
#include <srrg_pcl/instances.h>

#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg_solver/variables_and_factors/types_3d/instances.h>
#include <srrg_solver/variables_and_factors/types_projective/instances.h>

namespace srrg2_apriltag_calibration {

  void srrg2_apriltagCalibrator_registerTypes() {
    // ia other register class
    srrg2_core::point_cloud_registerTypes();
    srrg2_solver::linear_solver_registerTypes();
    srrg2_solver::solver_registerTypes();
    srrg2_solver::projective_registerTypes();
    srrg2_solver::registerTypes3D();
    srrg2_apriltagDetector_registerTypes();

    // ia actual classes
    BOSS_REGISTER_CLASS(ApriltagCalibrator36h11_OS164);
  }

} /* namespace srrg2_apriltag_calibration */
