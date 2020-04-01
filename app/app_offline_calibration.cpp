#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

#include <srrg_messages/instances.h>
#include <srrg_messages_ros/instances.h>
#include <srrg_pcl/instances.h>

#include <srrg_property/property_vector.h>

#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>

#include "srrg_apriltag_calibration/instances.h"

const std::string exe_name("app_offline_calibration");
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_solver;
using namespace srrg2_qgl_viewport;
using namespace srrg2_apriltag_calibration;

void generateConfig(const std::string& config_filename_);
void processFn(const ViewerCanvasPtr& canvas_,
               const std::string& dataset_file_,
               const std::string& config_file_,
               const std::string& output_file_,
               const std::string& dump_file_,
               const std::string& calib_name_,
               const std::string& source_name_);

int main(int argc, char** argv) {
  srrgInit(argc, argv, exe_name.c_str());
  messages_registerTypes();
  messages_ros_registerTypes();
  srrg2_apriltagCalibrator_registerTypes();
  Profiler::enable_logging = true;

  ParseCommandLine cmd(argv);
  ArgumentString param_dataset_filename(
    &cmd, "i", "input-dataset", "dataset filename either boss or bag", "");
  ArgumentString param_config_filename(
    &cmd, "c", "config", "config filename", "offline_calibrator.json");
  ArgumentString param_calibrator_name(
    &cmd, "cn", "calibrator-name", "calibrator name in the config", "calibrator_os1_64");
  ArgumentString param_output_filename(&cmd, "o", "output", "output file of the calibration", "");
  ArgumentString param_dump_filename(
    &cmd, "d", "dump-file", "dump file containing calibration entries", "");
  ArgumentString param_source_name(
    &cmd, "sn", "source-name", "source name in the config", "ros_source");
  ArgumentFlag param_generate_config(
    &cmd, "j", "generate-config", "true to generate default config", false);
  cmd.parse();

  if (param_generate_config.isSet()) {
    generateConfig(param_config_filename.value());
    LOG << "exit\n";
    return 0;
  }

  if (!param_dataset_filename.isSet()) {
    LOG << "ERROR, no dataset specified\n";
    std::cerr << cmd.options() << std::endl;
    throw std::runtime_error(exe_name + "|exit");
  }

  if (!checkFile(param_config_filename.value())) {
    throw std::runtime_error(exe_name + "|ERROR, invalid config file [ " +
                             param_config_filename.value() + " ]");
  }

  // ia start the thing
  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer_core(argc, argv, &qapp, BUFFER_SIZE_50MEGABYTE, 3, 25, true);
  const ViewerCanvasPtr& canvas_0 = viewer_core.getCanvas("detections");
  std::thread processing_t(processFn,
                           canvas_0,
                           param_dataset_filename.value(),
                           param_config_filename.value(),
                           param_output_filename.value(),
                           param_dump_filename.value(),
                           param_calibrator_name.value(),
                           param_source_name.value());
  viewer_core.startViewerServer();
  processing_t.join();
}

void processFn(const ViewerCanvasPtr& canvas_,
               const std::string& dataset_file_,
               const std::string& config_file_,
               const std::string& output_file_,
               const std::string& dump_file_,
               const std::string& calib_name_,
               const std::string& source_name_) {
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ConfigurableManager manager;
  manager.read(config_file_);

  // ia setup source
  MessageFileSourceBasePtr source   = manager.getByName<MessageFileSourceBase>(source_name_);
  MessageSynchronizedSourcePtr sync = manager.getByName<MessageSynchronizedSource>("sync");
  if (!source || !sync) {
    throw std::runtime_error(exe_name + "|cannot get sources");
  }
  LOG << "openening file [ " << dataset_file_ << " ]\n";
  source->open(dataset_file_);

  ApriltagCalibratorBasePtr calibrator = manager.getByName<ApriltagCalibratorBase>(calib_name_);
  LOG << "intial guess [x y z qx qy qz] = " << calibrator->param_inital_guess.value().transpose()
      << std::endl;

  BaseSensorMessagePtr msg;
  while ((msg = sync->getMessage()) && ViewerCoreSharedQGL::isRunning()) {
    // ia collect data from the measurements
    calibrator->setMessage(msg);

    // ia draw
    calibrator->draw(canvas_);
    canvas_->flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  source->close();

  // ia now we call the fuckin solver
  calibrator->compute();

  LOG << "final estimate [x y z qx qy qz] = "
      << geometry3d::t2tnq(calibrator->estimate()).transpose() << std::endl;

  if (!output_file_.empty()) {
    LOG << "saving final estimate from current intial guess to [ " << output_file_ << " ]\n";
    calibrator->save(output_file_);
  }

  if (!dump_file_.empty()) {
    LOG << "dumping calibration entries to [ " << dump_file_ << " ]\n";
    calibrator->dump(dump_file_);
  }

  std::cerr << "\n";
}

void generateConfig(const std::string& config_filename_) {
  ConfigurableManager manager;
  MessageROSBagSourcePtr ros_src    = manager.create<MessageROSBagSource>("ros_source");
  MessageFileSourcePtr boss_src     = manager.create<MessageFileSource>("boss_source");
  MessageSortedSourcePtr sorter     = manager.create<MessageSortedSource>("sorter");
  MessageSynchronizedSourcePtr sync = manager.create<MessageSynchronizedSource>("sync");

  ros_src->param_topics.pushBack("/os1_cloud_node/points");
  ros_src->param_topics.pushBack("/camera/image_rect");
  ros_src->param_topics.pushBack("/camera/camera_info");

  sorter->param_time_interval.setValue(1.0);
  sorter->param_source.setValue(ros_src);

  sync->param_time_interval.setValue(0.1);
  sync->param_source.setValue(sorter);
  sync->param_topics.pushBack("/os1_cloud_node/points");
  sync->param_topics.pushBack("/camera/image_rect");
  sync->param_topics.pushBack("/camera/camera_info");

  // ia setup projector
  PointIntensity3fProjectorOS1_64Ptr projector_ouster =
    manager.create<PointIntensity3fProjectorOS1_64>("projector_os1_64");
  projector_ouster->param_num_columns.setValue(1024);
  projector_ouster->param_horizontal_start_angle.setValue(2.f * M_PI);
  projector_ouster->param_horizontal_end_angle.setValue(0.0f);
  projector_ouster->param_range_max.setValue(15.f); // ia indoor

  // ia setup robustifier
  auto robustifier = manager.create<RobustifierCauchy>("cauchy_kernel");
  robustifier->param_chi_threshold.setValue(10.0);

  // ia create robustifier policy for 2D Pose-Bearing edges
  auto robustifier_policy = manager.create<RobustifierPolicyByType>("robustifier_policy");
  robustifier_policy->param_factor_class_name.setValue(
    "SE3ProjectiveErrorFactorCorrespondenceDriven");
  robustifier_policy->param_robustifier.setValue(robustifier);

  auto termination_criterium = manager.create<SimpleTerminationCriteria>("solver_termination_crit");
  termination_criterium->param_epsilon.setValue(1e-4);

  // ia setup solver
  SolverPtr solver = manager.create<Solver>("solver");
  solver->param_robustifier_policies.pushBack(RobustifierPolicyBasePtr(robustifier_policy));
  solver->param_termination_criteria.setValue(termination_criterium);
  solver->param_max_iterations.pushBack(25);

  // ia setup calibrator
  ApriltagCalibrator36h11_OS164Ptr calibrator =
    manager.create<ApriltagCalibrator36h11_OS164>("calibrator_os1_64");
  calibrator->param_lidar_projector.setValue(projector_ouster);
  calibrator->param_solver.setValue(solver);
  calibrator->param_max_intensity.setValue(250.f); // ia indoor
  calibrator->param_topic_lidar.setValue("/os1_cloud_node/points");
  calibrator->param_topic_camera.setValue("/camera/image_rect");
  calibrator->param_topic_camera_info.setValue("/camera/camera_info");

  LOG << "writing default configuration in [ " << config_filename_ << " ]\n";
  manager.write(config_filename_);
}
