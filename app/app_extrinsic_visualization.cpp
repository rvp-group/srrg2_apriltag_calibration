#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

#include <srrg_messages/instances.h>
#include <srrg_messages_ros/instances.h>
#include <srrg_pcl/instances.h>

#include "srrg_apriltag_calibration/instances.h"
#include "srrg_apriltag_detection/instances.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_apriltag_calibration;

const std::string exe_name("test_lidar_camera_projection");
#define LOG std::cerr << exe_name + "|"

// ia mega hack to keep the module clean :)
template <srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE lidar_type_>
class CalibratorDebug36h11_
  : public ApriltagCalibrator_<lidar_type_,
                               srrg2_apriltag_calibration::APRILTAG_FAMILY::Family_36h11> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline const CalibrationObservationPtrVector& lastObservations() const {
    return this->_current_calibration_observations;
  }

  inline cv::Mat cameraImageBgr() const {
    cv::Mat cv_image;
    this->_current_intensity_image_camera.toCv(cv_image);

    cv::cvtColor(cv_image, cv_image, CV_GRAY2BGR);
    return cv_image;
  }
};

using CalibratorDebug36h11OS1_64 =
  CalibratorDebug36h11_<srrg2_core::srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64>;
using CalibratorDebug36h11OS1_64Ptr = std::shared_ptr<CalibratorDebug36h11OS1_64>;

void generateConfig(const std::string& config_name_);

Vector6f readTransfromFromFile(const std::string& filename_);

int main(int argc, char** argv) {
  srrgInit(argc, argv, exe_name.c_str());
  messages_registerTypes();
  messages_ros_registerTypes();
  srrg2_apriltagCalibrator_registerTypes();
  BOSS_REGISTER_CLASS(CalibratorDebug36h11OS1_64);

  ParseCommandLine cmd(argv);
  ArgumentString param_config_filename(
    &cmd, "c", "config", "configiguration file", "config_extrinsic_visualization.json");
  ArgumentString param_dataset_filename(
    &cmd, "d", "dataset", "dataset filename either boss or bag", "");
  ArgumentString param_source_name(
    &cmd, "s", "source-name", "source name in the config", "ros_source");
  ArgumentString param_extrinsic_calibration_file(
    &cmd, "f", "calibration-file", "file containing the T_lidar_in_camera [x y z qx qy qz]", "");
  ArgumentString param_output_directory(
    &cmd, "o", "output-directory", "output directory where you want to save visualization", "");
  ArgumentFlag param_generate_config(
    &cmd, "j", "generate-config", "if it's set, generates default configuration", false);
  cmd.parse();

  if (param_generate_config.isSet()) {
    LOG << "generating default config\n";
    generateConfig(param_config_filename.value());
    LOG << "generated config file [ " << param_config_filename.value() << " ]\n";
    return 0;
  }

  // ia analyze cmd line arguments
  if (!param_dataset_filename.isSet()) {
    std::cerr << cmd.options();
    throw std::runtime_error(exe_name + "|ERROR, please specify dataset");
  }

  if (!param_extrinsic_calibration_file.isSet()) {
    std::cerr << cmd.options();
    throw std::runtime_error(exe_name + "|ERROR, please file with transform");
  }

  if (!checkFile(param_dataset_filename.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot find dataset [ " +
                             param_dataset_filename.value() + " ]");
  }

  if (!checkFile(param_extrinsic_calibration_file.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot find file [ " +
                             param_extrinsic_calibration_file.value() + " ]");
  }

  if (!checkFile(param_config_filename.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot find config [ " +
                             param_config_filename.value() + " ]");
  }

  Vector6f transformation_vector = readTransfromFromFile(param_extrinsic_calibration_file.value());
  //  transformation_vector << 0, -0.38618, -0.55, 0.5, 0.5, -0.5; // ia initial guess

  ConfigurableManager manager;
  manager.read(param_config_filename.value());

  // ia instanciate the readers
  auto source     = manager.getByName<MessageFileSourceBase>(param_source_name.value());
  auto sync       = manager.getByName<MessageSynchronizedSource>("sync");
  auto calibrator = manager.getByName<CalibratorDebug36h11OS1_64>("calibrator");
  if (!source || !sync || !calibrator) {
    throw std::runtime_error(exe_name + "|ERROR, cannot get modules");
  }

  // ia set transform
  calibrator->param_inital_guess.setValue(transformation_vector);

  srrg2_core::Isometry3f T =   srrg2_core::Isometry3f::Identity();
  T = srrg2_core::geometry3d::tnq2t(transformation_vector);
  std::cerr << "T \n" << T.matrix() << std::endl;

  // ia load dataset
  source->open(param_dataset_filename.value());
  BaseSensorMessagePtr msg;
  while ((msg = sync->getMessage())) {
    std::cerr << "\r" << exe_name << "|message [ " << FG_ULWHITE(msg->seq.value()) << " ]";

    // ia accumulate
    calibrator->setMessage(msg);

    if (!calibrator->lastObservations().size()) {
      continue;
    }

    cv::Mat cv_image = calibrator->cameraImageBgr();
    for (auto obs : calibrator->lastObservations()) {
      // ia draw the frame of the camera detection
      {
        const srrg2_core::Vector2f& c0 = obs->camera_points[0].coordinates();
        const srrg2_core::Vector2f& c1 = obs->camera_points[1].coordinates();
        const srrg2_core::Vector2f& c2 = obs->camera_points[2].coordinates();
        const srrg2_core::Vector2f& c3 = obs->camera_points[3].coordinates();
        cv::line(
          cv_image, cv::Point(c0.x(), c0.y()), cv::Point(c1.x(), c1.y()), cv::Scalar(0, 0, 255), 2);
        cv::line(
          cv_image, cv::Point(c0.x(), c0.y()), cv::Point(c3.x(), c3.y()), cv::Scalar(0, 0, 255), 2);
        cv::line(
          cv_image, cv::Point(c1.x(), c1.y()), cv::Point(c2.x(), c2.y()), cv::Scalar(0, 0, 255), 2);
        cv::line(
          cv_image, cv::Point(c2.x(), c2.y()), cv::Point(c3.x(), c3.y()), cv::Scalar(0, 0, 255), 2);
      }
      // ia draw same thing for the lidar
      {
        const srrg2_core::Vector2f& c0 = obs->lidar_points_transf_proj[0].coordinates();
        const srrg2_core::Vector2f& c1 = obs->lidar_points_transf_proj[1].coordinates();
        const srrg2_core::Vector2f& c2 = obs->lidar_points_transf_proj[2].coordinates();
        const srrg2_core::Vector2f& c3 = obs->lidar_points_transf_proj[3].coordinates();
        cv::line(
          cv_image, cv::Point(c0.x(), c0.y()), cv::Point(c1.x(), c1.y()), cv::Scalar(255, 0, 0), 2);
        cv::line(
          cv_image, cv::Point(c0.x(), c0.y()), cv::Point(c3.x(), c3.y()), cv::Scalar(255, 0, 0), 2);
        cv::line(
          cv_image, cv::Point(c1.x(), c1.y()), cv::Point(c2.x(), c2.y()), cv::Scalar(255, 0, 0), 2);
        cv::line(
          cv_image, cv::Point(c2.x(), c2.y()), cv::Point(c3.x(), c3.y()), cv::Scalar(255, 0, 0), 2);
      }
    }

    if (param_output_directory.isSet()) {
      if (!checkFile(param_output_directory.value())) {
        throw std::runtime_error(exe_name + "|ERROR, output folder [ " +
                                 param_output_directory.value() + " ] does not exist");
      }

      const std::string filename =
        param_output_directory.value() + "/" + std::to_string(msg->timestamp.value()) + ".png";
      cv::imwrite(filename, cv_image);
    }

    cv::imshow("registered detections", cv_image);
    cv::waitKey(0);
  }
}

// ia config generator for this app
void generateConfig(const std::string& config_name_) {
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

  CalibratorDebug36h11OS1_64Ptr calibrator =
    manager.create<CalibratorDebug36h11OS1_64>("calibrator");
  calibrator->param_lidar_projector.setValue(projector_ouster);
  calibrator->param_max_intensity.setValue(250.f); // ia indoor
  calibrator->param_topic_lidar.setValue("/os1_cloud_node/points");
  calibrator->param_topic_camera.setValue("/camera/image_rect");
  calibrator->param_topic_camera_info.setValue("/camera/camera_info");

  manager.write(config_name_);
}

Vector6f readTransfromFromFile(const std::string& filename_) {
  Vector6f transform = Vector6f::Zero();

  std::string line;
  std::ifstream f_stream(filename_);

  while (getline(f_stream, line)) {
    std::stringstream ss(line);
    std::string tag;
    ss >> tag;
    if (tag != "ESTIMATE")
      continue;
    ss >> transform[0] >> transform[1] >> transform[2] >> transform[3] >> transform[4] >>
      transform[5];
  }

  f_stream.close();
  LOG << "transform loaded [ " << transform.transpose() << " ]\n";

  return transform;
}
