#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

class JointStateMerger : public rclcpp::Node
{
public:
  JointStateMerger()
  : Node("joint_state_merger")
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/isaac_joint_commands");
    state_topic_ = this->declare_parameter<std::string>(
      "state_topic", "/isaac_joint_states");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/isaac_joint_commands_merged");

    // Only the joints Isaac should actually command.
    joint_order_ = this->declare_parameter<std::vector<std::string>>(
      "joint_order",
      {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
        "robotiq_85_left_knuckle_joint",
        "robotiq_85_right_knuckle_joint"
      });

    index_of_.reserve(joint_order_.size());
    for (size_t i = 0; i < joint_order_.size(); ++i) {
      index_of_[joint_order_[i]] = i;
    }

    // Start uninitialized. We seed from /isaac_joint_states.
    targets_.assign(joint_order_.size(), 0.0);
    initialized_.assign(joint_order_.size(), false);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic_, qos);

    cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, qos,
      std::bind(&JointStateMerger::command_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&JointStateMerger::state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Subscribed to '%s' and '%s', publishing merged state to '%s'",
      input_topic_.c_str(), state_topic_.c_str(), output_topic_.c_str());
  }

private:
  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Seed unknown joints from the current robot state.
    // This prevents gripper-only commands from forcing the arm to zero.
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto it = index_of_.find(msg->name[i]);
      if (it == index_of_.end()) {
        continue;
      }

      const size_t idx = it->second;
      if (i < msg->position.size()) {
        targets_[idx] = msg->position[i];
        initialized_[idx] = true;
      }
    }

    have_state_seed_ = true;
  }

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Do not publish until we have at least one state sample to seed missing joints.
    if (!have_state_seed_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for initial joint state seed on '%s'...", state_topic_.c_str());
      return;
    }

    // Update only the joints present in this command message.
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto it = index_of_.find(msg->name[i]);
      if (it == index_of_.end()) {
        continue;
      }

      const size_t idx = it->second;
      if (i < msg->position.size()) {
        targets_[idx] = msg->position[i];
        initialized_[idx] = true;
      }
    }

    publish_merged_locked();
  }

  void publish_merged_locked()
  {
    sensor_msgs::msg::JointState out;
    out.header.stamp = this->now();
    out.name = joint_order_;
    out.position = targets_;

    // Keep these the same size as name/position for downstream consumers.
    out.velocity.assign(joint_order_.size(), 0.0);
    out.effort.assign(joint_order_.size(), 0.0);

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string state_topic_;
  std::string output_topic_;

  std::vector<std::string> joint_order_;
  std::unordered_map<std::string, size_t> index_of_;

  std::vector<double> targets_;
  std::vector<bool> initialized_;
  bool have_state_seed_{false};

  std::mutex mutex_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateMerger>());
  rclcpp::shutdown();
  return 0;
}