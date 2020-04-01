add_library(srrg2_apriltag_calibration_library SHARED
  apriltag_calibrator.h
  apriltag_calibrator.hpp
  instances.h
  instances.cpp)
target_link_libraries(srrg2_apriltag_calibration_library
  srrg2_apriltag_detection_library 
  srrg2_solver_core_library
  srrg2_solver_linear_solvers_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library)
