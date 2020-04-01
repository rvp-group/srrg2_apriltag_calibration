#pragma once
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <srrg_messages/instances.h>

namespace srrg2_apriltag_calibration {

  // ia checks whether file exists
  inline bool checkFile(const std::string& name_) {
    struct stat buffer;
    return (stat(name_.c_str(), &buffer) == 0);
  }

  template <typename DestMessageType_>
  std::shared_ptr<DestMessageType_> extractMessage(srrg2_core::BaseSensorMessagePtr measurement,
                                                   const std::string& topic_) {
    srrg2_core::MessagePackPtr message_pack =
      std::dynamic_pointer_cast<srrg2_core::MessagePack>(measurement);

    // ia if it's not a pack, try to process a single laser message
    if (!message_pack) {
      if (measurement->topic.value() == topic_) {
        return std::dynamic_pointer_cast<DestMessageType_>(measurement);
      }
      return std::shared_ptr<DestMessageType_>();
    }
    // is a pack
    for (srrg2_core::BaseSensorMessagePtr& message : message_pack->messages) {
      if (message->topic.value() != topic_) {
        continue;
      }
      std::shared_ptr<DestMessageType_> result =
        std::dynamic_pointer_cast<DestMessageType_>(message);
      if (result) {
        return result;
      }
    }
    return nullptr;
  }
} // namespace srrg2_apriltag_calibration
