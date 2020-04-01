add_library(srrg2_apriltag_detection_library SHARED
  apriltag.h
  apriltag.hpp
  apriltag_detector.h
  apriltag_detector.hpp
  instances.h
  instances.cpp)
target_link_libraries(srrg2_apriltag_detection_library
  srrg2_apriltag3_library
  srrg2_config_library
  srrg2_image_library)
