#ia GTEST: detector
catkin_add_gtest(test_detector test_detector.cpp)
target_link_libraries(test_detector
  srrg2_apriltag_detection_library
  ${catkin_LIBRARIES})
