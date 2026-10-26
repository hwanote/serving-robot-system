// Evidence only. Never broadcasts TF or feeds localization/control.
#include <memory>
#include <string>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class TruthRelay : public rclcpp::Node {
 public:
  TruthRelay() : rclcpp::Node("gazebo_truth_evidence") {
    pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/cafe/gazebo_pose_truth", 10);
    transport_.Subscribe("/world/cafe/dynamic_pose/info", &TruthRelay::receive, this);
  }
 private:
  void receive(const gz::msgs::Pose_V &message) {
    tf2_msgs::msg::TFMessage output;
    for (const auto &pose : message.pose()) {
      if (pose.name() != "robot_01" && pose.name() != "robot_02" &&
          pose.name().rfind("test_person", 0) != 0 &&
          pose.name().rfind("dynamic_", 0) != 0 &&
          pose.name().rfind("moving_obstacle", 0) != 0) continue;
      geometry_msgs::msg::TransformStamped value;
      value.header.frame_id = "world";
      value.header.stamp.sec = message.header().stamp().sec();
      value.header.stamp.nanosec = message.header().stamp().nsec();
      value.child_frame_id = pose.name();
      value.transform.translation.x = pose.position().x();
      value.transform.translation.y = pose.position().y();
      value.transform.translation.z = pose.position().z();
      value.transform.rotation.x = pose.orientation().x();
      value.transform.rotation.y = pose.orientation().y();
      value.transform.rotation.z = pose.orientation().z();
      value.transform.rotation.w = pose.orientation().w();
      output.transforms.push_back(value);
    }
    if (!output.transforms.empty()) pub_->publish(output);
  }
  gz::transport::Node transport_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TruthRelay>());
  rclcpp::shutdown();
  return 0;
}
