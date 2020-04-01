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

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_lidar3d_utils;

const std::string exe_name("app_image_dumper");
#define LOG std::cerr << exe_name + "|"

// ia checks whether file exists
inline bool checkFile(const std::string& name_) {
  struct stat buffer;
  return (stat(name_.c_str(), &buffer) == 0);
}

void generateConfig(const std::string& config_name_);

// using LidarProjectorType    = PointIntensity3fProjectorHDL64E;
// using LidarUnprojectorType  = PointIntensity3fUnprojectorHDL64E;
using LidarProjectorType   = PointIntensity3fProjectorOS1_64;
using LidarUnprojectorType = PointIntensity3fUnprojectorOS1_64;
using NormalComputatorType = NormalComputator2DCrossProduct<PointNormal3fMatrixCloud, 1>;

void dumpLidar(LidarProjectorType& projector_,
               const PointCloud2MessagePtr& lidar_msg_,
               const std::string& directory_,
               const bool dump_depth_);
void dumpCamera(const ImageMessagePtr& image_msg_, const std::string& directory_);

cv::Mat cv_intensity_image, cv_depth_image, cv_camera_image;
size_t msg_number = 0;

int main(int argc, char** argv) {
  // bdc register messages type
  srrgInit(argc, argv, exe_name.c_str());
  messages_registerTypes();
  messages_ros_registerTypes();

  ParseCommandLine cmd(argv);
  ArgumentString config_filename(&cmd, "c", "config", "configiguration file", "image_dumper.conf");
  ArgumentString source_name(&cmd, "sn", "source-name", "source name in the config", "ros_source");
  ArgumentFlag generate_config(
    &cmd, "j", "generate-config", "if it's set, generates default configuration", false);
  ArgumentString output_lidar_directory(
    &cmd, "ol", "output-lidar", "directory where lidar intensity images will be saved", "");
  ArgumentString output_camera_directory(
    &cmd, "oc", "output-camera", "directory where camera images will be saved", "");
  ArgumentString dataset_filename(&cmd, "d", "dataset", "dataset filename either boss or bag", "");
  ArgumentDouble lidar_projector_min_depth(
    &cmd, "min-d", "min-depth", "lidar projector min depth [m]", 0.3);
  ArgumentDouble lidar_projector_max_depth(
    &cmd, "max-d", "max-depth", "lidar projector max depth [m]", 100.0);
  ArgumentDouble lidar_projector_horizontal_start_angle(
    &cmd, "hs", "horizonal-start", "horizontal starting angle for lidar projection [rad]", M_PI);
  ArgumentDouble lidar_projector_horizontal_end_angle(
    &cmd, "he", "horizonal-end", "horizontal ending angle for projection [rad]", -M_PI);
  ArgumentInt lidar_projector_num_columns(
    &cmd, "col", "column", "lidar projected image column", 1024);
  ArgumentFlag dump_lidar_depth_images(
    &cmd, "dd", "dump-depth", "if it's set, dumps also the lidar depth image", false);
  cmd.parse();

  if (generate_config.isSet()) {
    LOG << "generating default config\n";
    generateConfig(config_filename.value());
    LOG << "done!\n";
    return 0;
  }

  if (!dataset_filename.isSet() || !output_lidar_directory.isSet() ||
      !output_camera_directory.isSet()) {
    std::cerr << cmd.options();
    throw std::runtime_error(
      exe_name + "|ERROR, invalid parameters, please specify output directories and dataset");
  }

  if (!checkFile(output_lidar_directory.value()) || !checkFile(output_camera_directory.value())) {
    throw std::runtime_error(exe_name + "|ERROR, invalid output directories");
  }

  if (!checkFile(config_filename.value())) {
    throw std::runtime_error(exe_name + "|ERROR, invalid configuration file [ " +
                             config_filename.value() + " ]");
  }

  // ia read configuration for the source
  ConfigurableManager manager;
  manager.read(config_filename.value());

  MessageFileSourceBasePtr source   = manager.getByName<MessageFileSourceBase>(source_name.value());
  MessageSynchronizedSourcePtr sync = manager.getByName<MessageSynchronizedSource>("sync");
  if (!source || !sync) {
    throw std::runtime_error(exe_name + "|cannot get sources");
  }
  source->open(dataset_filename.value());

  // ia setup projector
  LidarProjectorType projector;
  projector.param_num_columns.setValue(lidar_projector_num_columns.value());
  projector.param_range_min.setValue(lidar_projector_min_depth.value());
  projector.param_range_max.setValue(lidar_projector_max_depth.value());
  projector.param_horizontal_start_angle.setValue(lidar_projector_horizontal_start_angle.value());
  projector.param_horizontal_end_angle.setValue(lidar_projector_horizontal_end_angle.value());

  // ia read and process the thing
  BaseSensorMessagePtr msg;
  while ((msg = sync->getMessage())) {
    // ia we need a pack
    MessagePackPtr msg_pack = std::dynamic_pointer_cast<MessagePack>(msg);
    if (!msg_pack || msg_pack->messages.size() < 2) {
      throw std::runtime_error(exe_name + "|dataset should contain at least camera and lidar data");
    }

    for (size_t i = 0; i < msg_pack->messages.size(); ++i) {
      PointCloud2MessagePtr lidar_msg =
        std::dynamic_pointer_cast<PointCloud2Message>(msg_pack->messages.at(i));
      ImageMessagePtr camera_msg =
        std::dynamic_pointer_cast<ImageMessage>(msg_pack->messages.at(i));
      if (lidar_msg) {
        dumpLidar(
          projector, lidar_msg, output_lidar_directory.value(), dump_lidar_depth_images.isSet());
      } else if (camera_msg) {
        dumpCamera(camera_msg, output_camera_directory.value());
      } else {
        continue;
      }
    }

    std::cerr << "\rprocessing message [ " << msg_number++ << " ]";
    cv::imshow("lidar_depth", cv_depth_image);
    cv::imshow("lidar_intensity", cv_intensity_image);
    cv::imshow("camera_grayscale", cv_camera_image);
    cv::waitKey(1);
  }

  return 0;
}

void dumpLidar(LidarProjectorType& projector_,
               const PointCloud2MessagePtr& lidar_msg_,
               const std::string& directory_,
               const bool dump_depth_) {
  LidarProjectorType::Lidar3DSensorType lidar_sensor;
  // ia projected point cloud (within a special data structure containing auxliary fields also)
  LidarProjectorType::TargetMatrixType projection_target_matrix;
  // ia intensity and depth image from the projected cloud
  srrg2_core::ImageFloat intensity_image;
  srrg2_core::ImageFloat depth_image;

  // ia extract a usable pointcloud from the raw message
  PointIntensity3fVectorCloud current_cloud;
  lidar_msg_->getPointCloud(current_cloud);

  float max_intensity = -1.f;
  for (const auto& p : current_cloud) {
    if (p.intensity() > max_intensity) {
      max_intensity = p.intensity();
    }
  }

  // ia project the cloud
  projector_.compute(projection_target_matrix, current_cloud.begin(), current_cloud.end());

  // ia get the intensity image
  intensity_image.resize(lidar_sensor.verticalResolution(), projector_.param_num_columns.value());
  auto intensity_image_it = intensity_image.begin();
  for (auto it = projection_target_matrix.begin(); it != projection_target_matrix.end();
       ++it, ++intensity_image_it) {
    *intensity_image_it = it->source_it->intensity();
  }

  // ia get the depth image
  projection_target_matrix.toDepthMatrix(depth_image);

  // ia use opencv to show something
  intensity_image.toCv(cv_intensity_image);
  depth_image.toCv(cv_depth_image);

  // ia save the intensity image in the output directory
  cv::imwrite(directory_ + "/intensity_" + std::to_string(msg_number) + ".png", cv_intensity_image);
  if (dump_depth_) {
    cv::imwrite(directory_ + "/depth_" + std::to_string(msg_number) + ".png", cv_depth_image);
  }

  cv_depth_image     = cv_depth_image / projector_.param_range_max.value();
  cv_intensity_image = cv_intensity_image / max_intensity;
}

void dumpCamera(const ImageMessagePtr& image_msg_, const std::string& directory_) {
  cv::Mat temp_image;
  image_msg_->image()->toCv(temp_image);
  cv::cvtColor(temp_image, cv_camera_image, CV_BGR2GRAY);
  cv::imwrite(directory_ + "/camera_" + std::to_string(msg_number) + ".png", cv_camera_image);
}

void generateConfig(const std::string& config_filename_) {
  ConfigurableManager manager;
  MessageROSBagSourcePtr ros_src    = manager.create<MessageROSBagSource>("ros_source");
  MessageFileSourcePtr boss_src     = manager.create<MessageFileSource>("boss_source");
  MessageSortedSourcePtr sorter     = manager.create<MessageSortedSource>("sorter");
  MessageSynchronizedSourcePtr sync = manager.create<MessageSynchronizedSource>("sync");

  ros_src->param_topics.pushBack("/os1_cloud_node/points");
  ros_src->param_topics.pushBack("/camera/image_raw");

  sorter->param_time_interval.setValue(1.0);
  sorter->param_source.setValue(ros_src);

  sync->param_time_interval.setValue(0.1);
  sync->param_source.setValue(sorter);
  sync->param_topics.pushBack("/os1_cloud_node/points");
  sync->param_topics.pushBack("/camera/image_raw");

  LOG << "writing configuration in [ " << config_filename_ << " ]\n";
  manager.write(config_filename_);
}
