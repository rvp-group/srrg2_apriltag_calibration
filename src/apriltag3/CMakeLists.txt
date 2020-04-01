# inlude_directories(.)
aux_source_directory(common COMMON_SRC)
set(APRILTAG_SRCS apriltag.c apriltag_pose.c apriltag_quad_thresh.c)

# Library
file(GLOB TAG_FILES
  tag16h5.c
  tag16h5.h
  tag25h9.c
  tag25h9.h
  tag36h11.c
  tag36h11.h
  tagCircle21h7.c
  tagCircle21h7.h
  tagCircle49h12.c
  tagCircle49h12.h
  tagCustom48h12.c
  tagCustom48h12.h
  tagStandard41h12.c
  tagStandard41h12.h
  tagStandard52h13.c
  tagStandard52h13.h)
set_source_files_properties(SOURCE ${TAG_FILES} PROPERTIES COMPILE_FLAGS -O0)
add_library(srrg2_apriltag3_library SHARED ${APRILTAG_SRCS} ${COMMON_SRC} ${TAG_FILES})
target_link_libraries(srrg2_apriltag3_library Threads::Threads m)
set_target_properties(srrg2_apriltag3_library PROPERTIES SOVERSION 3 VERSION 3.1.0)
