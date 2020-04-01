cmake_minimum_required(VERSION 2.8.3)
project(srrg2_apriltag_calibration)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  srrg2_core_ros
  srrg2_solver
  srrg2_qgl_viewport
  )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(GLUT REQUIRED)
message("${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION}")
message("${PROJECT_NAME}|OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")


catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_apriltag3_library
    srrg2_apriltag_detection_library
    srrg2_apriltag_calibration_library
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg2_core
    srrg2_core_ros
    srrg2_solver
    srrg2_qgl_viewport
  DEPENDS
    OpenCV
    )

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${QGLViewer_INCLUDE_DIR}
  ${SRRG_QT_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

link_directories(${catkin_LIBRARY_DIRS})

add_definitions(-DSRRG2_APRILTAG_CALIBRATION_TEST_DATA_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
message("${PROJECT_NAME}|test_data dir = ${CMAKE_CURRENT_SOURCE_DIR}/test_data/")

# set sources
add_subdirectory(src)
add_subdirectory(app)
add_subdirectory(tests)
