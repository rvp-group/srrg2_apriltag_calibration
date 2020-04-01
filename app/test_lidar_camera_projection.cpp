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

#include "srrg_apriltag_detection/apriltag_detector.h"
#include "srrg_apriltag_utils/utilities.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;

const std::string exe_name("test_lidar_camera_projection");
#define LOG std::cerr << exe_name + "|"

// ia checks whether file exists
inline bool checkFile(const std::string& name_) {
  struct stat buffer;
  return (stat(name_.c_str(), &buffer) == 0);
}

void generateConfig(const std::string& config_name_,
                    const std::string& topic_lidar_,
                    const std::string& topic_camera_,
                    const std::string& topic_camera_info_);

Vector6f readTransfromFromFile(const std::string& filename_);

Vector2f pinholeProject(const Vector3f& p,
                        const Matrix3f& K_,
                        const Isometry3f& T_,
                        const size_t& cols_,
                        const size_t& rows_);

int main(int argc, char** argv) {
  srrgInit(argc, argv, exe_name.c_str());
  point_cloud_registerTypes();
  messages_registerTypes();
  messages_ros_registerTypes();

  ParseCommandLine cmd(argv);
  ArgumentString param_config_filename(
    &cmd, "c", "config", "configiguration file", "config_lidar_camera_projector.json");
  ArgumentString param_dataset_filename(
    &cmd, "d", "dataset", "dataset filename either boss or bag", "");
  ArgumentString param_source_name(
    &cmd, "s", "source-name", "source name in the config", "ros_source");
  ArgumentString param_output_directory(
    &cmd, "o", "output", "set this if you want to save the images", "");
  ArgumentString param_topic_lidar(
    &cmd, "tl", "topic-lidar", "lidar pointcloud topic name", "/os1_cloud_node/points");
  ArgumentString param_topic_camera(
    &cmd, "tc", "topic-camera", "rectified image topic name", "/camera/image_rect");
  ArgumentString param_topic_camera_info(
    &cmd, "tci", "topic-camera-info", "camera info topic name", "/camera/camera_info");
  ArgumentString param_extrinsic_calibration_file(
    &cmd, "f", "calibration-file", "file containing the T_lidar_in_camera [x y z qx qy qz]", "");
  ArgumentFlag param_generate_config(
    &cmd, "j", "generate-config", "if it's set, generates default configuration", false);
  cmd.parse();

  if (param_generate_config.isSet()) {
    LOG << "generating default config\n";
    generateConfig(param_config_filename.value(),
                   param_topic_lidar.value(),
                   param_topic_camera.value(),
                   param_topic_camera_info.value());
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

  const Isometry3f T_lidar_in_camera = geometry3d::v2t(transformation_vector);

  ConfigurableManager manager;
  manager.read(param_config_filename.value());

  // ia instanciate the readers
  auto source = manager.getByName<MessageFileSourceBase>(param_source_name.value());
  auto sync   = manager.getByName<MessageSynchronizedSource>("sync");
  if (!source || !sync) {
    throw std::runtime_error(exe_name + "|ERROR, cannot get sources");
  }

  // ia load dataset
  source->open(param_dataset_filename.value());
  BaseSensorMessagePtr msg;
  while ((msg = sync->getMessage())) {
    std::cerr << "\r" << exe_name << "|message [ " << FG_ULWHITE(msg->seq.value()) << " ]";

    auto lidar_msg_ptr = srrg2_apriltag_calibration::extractMessage<PointCloud2Message>(
      msg, param_topic_lidar.value());
    auto image_msg_ptr =
      srrg2_apriltag_calibration::extractMessage<ImageMessage>(msg, param_topic_camera.value());
    auto camera_info_msg_ptr = srrg2_apriltag_calibration::extractMessage<CameraInfoMessage>(
      msg, param_topic_camera_info.value());

    if (!(lidar_msg_ptr && image_msg_ptr && camera_info_msg_ptr)) {
      throw std::runtime_error(
        exe_name + "|ERROR, you need lidar points, camera and camera info to run this sickness");
    }

    // ia take the image
    cv::Mat cv_black(image_msg_ptr->image()->rows(), image_msg_ptr->image()->cols(), CV_8UC3);
    cv_black.setTo(cv::Scalar(0, 0, 0));
    cv::Mat cv_image;
    image_msg_ptr->image()->toCv(cv_image);
    cv::cvtColor(cv_image, cv_image, CV_RGB2BGR);

    // ia take camera matrix
    const Matrix3f& K = camera_info_msg_ptr->camera_matrix.value();

    // ia take the point cloud and transform it according the T
    PointIntensity3fVectorCloud src_cloud;
    lidar_msg_ptr->getPointCloud(src_cloud);
    src_cloud.transformInPlace(T_lidar_in_camera);

    // ia project the points onto the camera
    PointIntensity2fVectorCloud projected_cloud;
    projected_cloud.reserve(src_cloud.size());
    for (const auto& p : src_cloud) {
      PointIntensity2f pp;
      pp.coordinates() =
        pinholeProject(p.coordinates(), K, Isometry3f::Identity(), cv_image.cols, cv_image.rows);

      // ia discard points
      if (pp.coordinates().x() < 0 || pp.coordinates().y() < 0) {
        continue;
      }

      pp.intensity() = p.intensity();
      projected_cloud.emplace_back(pp);

      // ia use this loop also to draw
      cv::Point2f cv_pp(pp.coordinates().x(), pp.coordinates().y());
      cv::circle(
        cv_image, cv_pp, 1, cv::Scalar(p.intensity(), p.intensity(), p.intensity(), 0.9), 1);
      //      cv::circle(cv_black, cv_pp, 1, cv::Scalar(p.intensity(), p.intensity(),
      //      p.intensity()), 1);
      cv::circle(cv_black,
                 cv_pp,
                 1,
                 cv::Scalar(cv_image.at<cv::Vec3b>(cv_pp).val[0],
                            cv_image.at<cv::Vec3b>(cv_pp).val[1],
                            cv_image.at<cv::Vec3b>(cv_pp).val[2]),
                 1);
    }

    cv::imshow("lidar-points-in-camera", cv_image);
    cv::imshow("lidar-points", cv_black);
    cv::waitKey(0);
  }
}

// ia config generator for this app
void generateConfig(const std::string& config_name_,
                    const std::string& topic_lidar_,
                    const std::string& topic_camera_,
                    const std::string& topic_camera_info_) {
  ConfigurableManager manager;
  MessageROSBagSourcePtr ros_src    = manager.create<MessageROSBagSource>("ros_source");
  MessageFileSourcePtr boss_src     = manager.create<MessageFileSource>("boss_source");
  MessageSortedSourcePtr sorter     = manager.create<MessageSortedSource>("sorter");
  MessageSynchronizedSourcePtr sync = manager.create<MessageSynchronizedSource>("sync");

  ros_src->param_topics.pushBack(topic_lidar_);
  ros_src->param_topics.pushBack(topic_camera_);
  ros_src->param_topics.pushBack(topic_camera_info_);

  sorter->param_time_interval.setValue(1.0);
  sorter->param_source.setValue(ros_src);

  sync->param_time_interval.setValue(0.1);
  sync->param_source.setValue(sorter);
  sync->param_topics.pushBack(topic_lidar_);
  sync->param_topics.pushBack(topic_camera_);
  sync->param_topics.pushBack(topic_camera_info_);

  manager.write(config_name_);
}

// ia aux functions
Vector2f pinholeProject(const Vector3f& p,
                        const Matrix3f& K_,
                        const Isometry3f& T_,
                        const size_t& cols_,
                        const size_t& rows_) {
  srrg2_core::Vector2f pp = srrg2_core::Vector2f::Zero();
  pp << -1.f, -1.f;

  const srrg2_core::Vector3f p_transf = T_ * p;
  if (p_transf.z() < 0) {
    return pp;
  }

  const float iz                    = 1. / p_transf.z();
  const srrg2_core::Vector3f cam_pt = K_ * p_transf;
  const srrg2_core::Vector2f img_pt(cam_pt.x() * iz, cam_pt.y() * iz);

  if (img_pt.x() < 0 || img_pt.x() > cols_) {
    return pp;
  }
  if (img_pt.y() < 0 || img_pt.y() > rows_) {
    return pp;
  }

  pp = img_pt;
  return pp;
}

Vector6f readTransfromFromFile(const std::string& filename_) {
  Vector6f transform = Vector6f::Zero();

  std::string line;
  std::ifstream f_stream(filename_);

  getline(f_stream, line);
  std::stringstream ss(line);
  ss >> transform[0] >> transform[1] >> transform[2] >> transform[3] >> transform[4] >>
    transform[5];

  f_stream.close();
  LOG << "transform loaded [ " << transform.transpose() << " ]\n";

  return transform;
}
