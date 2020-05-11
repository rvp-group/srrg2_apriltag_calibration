add_executable(app_image_dumper app_image_dumper.cpp)
target_link_libraries(app_image_dumper
  srrg2_apriltag_detection_library
  ${catkin_LIBRARIES})

#ia TODO, also a unit test would be ok
add_executable(app_offline_calibration app_offline_calibration.cpp)
target_link_libraries(app_offline_calibration
  srrg2_apriltag_detection_library
  srrg2_apriltag_calibration_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES})
  
#ia just to show the result of the calibration
add_executable(app_test_lidar_camera_projection test_lidar_camera_projection.cpp)
target_link_libraries(app_test_lidar_camera_projection
  srrg2_apriltag_detection_library
  srrg2_apriltag_calibration_library
  ${catkin_LIBRARIES})
  
  
#ia TODO, also a unit test would be ok
add_executable(app_extrinsic_visualization app_extrinsic_visualization.cpp)
target_link_libraries(app_extrinsic_visualization
  srrg2_apriltag_detection_library
  srrg2_apriltag_calibration_library
  ${catkin_LIBRARIES})
