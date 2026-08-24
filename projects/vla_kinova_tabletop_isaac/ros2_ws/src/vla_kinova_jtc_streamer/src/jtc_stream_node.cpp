// Minimal reliable-timing trajectory streamer for the Kinova JTC (passthrough, linear).
//
// The Python VLA client publishes a 30 Hz "ideal" arm plan as a JointTrajectory on
// `plan_topic` (header.stamp = the ROS time point 0 starts; point k stamped at
// k/control_hz). At `tick_hz` this node linearly interpolates the latest plan at the
// current time and streams ONE point to the JTC with time_from_start = `jtc_horizon`;
// the JTC's own 1 kHz interpolation fills in between. Deterministic C++ timing, no GIL.
//
// Two knobs: tick_hz (stream rate) and jtc_horizon (latency <-> smoothness). Passthrough
// only -- no jerk limiter and no cubic/quintic. (The jerk-limited / multi-interp research
// version lives in the vla_traj_streamer package on the devel branch.)

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using Vec6 = std::array<double, 6>;

static const std::array<std::string, 6> ARM_JOINTS =
{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

static builtin_interfaces::msg::Duration dur_from_sec(double s)
{
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(s);
  d.nanosec = static_cast<uint32_t>((s - d.sec) * 1e9);
  return d;
}

static double tfs_sec(const builtin_interfaces::msg::Duration & d)
{
  return d.sec + d.nanosec * 1e-9;
}

class JtcStreamNode : public rclcpp::Node
{
public:
  JtcStreamNode()
  : Node("jtc_stream_node")
  {
    tick_hz_ = std::max(declare_parameter<double>("tick_hz", 300.0), 1.0);
    jtc_horizon_ = std::max(declare_parameter<double>("jtc_horizon", 0.1), 1e-3);
    control_hz_ = declare_parameter<double>("control_hz", 30.0);   // fallback knot spacing
    std::string plan_topic = declare_parameter<std::string>("plan_topic", "/vla_arm_plan");
    std::string arm_topic = declare_parameter<std::string>(
      "arm_trajectory_topic", "/joint_trajectory_controller/joint_trajectory");

    tick_period_ = 1.0 / tick_hz_;
    dt_default_ = 1.0 / control_hz_;

    arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      arm_topic, rclcpp::QoS(rclcpp::KeepLast(1)));
    plan_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      plan_topic, rclcpp::QoS(rclcpp::KeepLast(2)),
      std::bind(&JtcStreamNode::on_plan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "jtc_stream_node | tick %.0f Hz | linear passthrough | jtc_horizon %.0f ms\n"
      "  plan<-'%s'  arm->'%s'",
      tick_hz_, jtc_horizon_ * 1e3, plan_topic.c_str(), arm_topic.c_str());

    running_ = true;
    thread_ = std::thread(&JtcStreamNode::stream_loop, this);
  }

  ~JtcStreamNode() override { stop(); }
  void stop()
  {
    running_ = false;
    if (thread_.joinable()) thread_.join();
  }

private:
  double now_sec() { return this->now().nanoseconds() * 1e-9; }

  // Latest 30 Hz plan from the Python client (replaces the active one).
  void on_plan(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    const size_t n = msg->points.size();
    if (n == 0) return;
    // Map the message joint order onto ARM_JOINTS (the plan may arrive in any order).
    std::array<int, 6> idx{-1, -1, -1, -1, -1, -1};
    for (size_t j = 0; j < msg->joint_names.size(); ++j) {
      for (int a = 0; a < 6; ++a) {
        if (msg->joint_names[j] == ARM_JOINTS[a]) idx[a] = static_cast<int>(j);
      }
    }
    for (int a = 0; a < 6; ++a) {
      if (idx[a] < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "plan missing joint %s; ignoring", ARM_JOINTS[a].c_str());
        return;
      }
    }
    std::vector<double> arr(n * 6);
    for (size_t k = 0; k < n; ++k) {
      const auto & pos = msg->points[k].positions;
      for (int a = 0; a < 6; ++a) arr[k * 6 + a] = pos[idx[a]];
    }
    // Knot spacing from the point stamps (fallback to control_hz); t_ref from
    // header.stamp (fallback to receive time if the publisher left it at 0).
    double dt = dt_default_;
    if (n >= 2) {
      double d = tfs_sec(msg->points[1].time_from_start) - tfs_sec(msg->points[0].time_from_start);
      if (d > 1e-6) dt = d;
    }
    double t_ref = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if (t_ref <= 0.0) t_ref = now_sec();

    std::lock_guard<std::mutex> lk(plan_mtx_);
    plan_ = std::move(arr);
    plan_n_ = static_cast<int>(n);
    plan_dt_ = dt;
    plan_t_ref_ = t_ref;
    have_plan_ = true;
  }

  // Linear interpolation of the arm columns at fractional index f, clamped to the plan.
  static Vec6 sample_linear(const std::vector<double> & plan, int n, double f)
  {
    Vec6 pos{};
    if (n <= 1) {
      for (int a = 0; a < 6; ++a) pos[a] = plan[a];
      return pos;
    }
    if (f < 0.0) f = 0.0;
    if (f > n - 1) f = n - 1;
    int i = static_cast<int>(std::floor(f));
    if (i > n - 2) i = n - 2;
    if (i < 0) i = 0;
    double u = f - i;
    for (int a = 0; a < 6; ++a) {
      double p1 = plan[static_cast<size_t>(i) * 6 + a];
      double p2 = plan[static_cast<size_t>(i + 1) * 6 + a];
      pos[a] = (1.0 - u) * p1 + u * p2;
    }
    return pos;
  }

  void publish_point(const Vec6 & pos)
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names.assign(ARM_JOINTS.begin(), ARM_JOINTS.end());
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(pos.begin(), pos.end());
    pt.time_from_start = dur_from_sec(jtc_horizon_);
    msg.points.push_back(std::move(pt));
    arm_pub_->publish(std::move(msg));
  }

  void stream_loop()
  {
    auto next = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(tick_period_);
    long n_report = 0;
    auto report_wall = std::chrono::steady_clock::now();

    while (running_ && rclcpp::ok()) {
      std::vector<double> plan; int pn = 0; double pdt = 0, pref = 0; bool have;
      {
        std::lock_guard<std::mutex> lk(plan_mtx_);
        have = have_plan_;
        if (have) { plan = plan_; pn = plan_n_; pdt = plan_dt_; pref = plan_t_ref_; }
      }

      if (have) {
        double f = (now_sec() - pref) / pdt;
        publish_point(sample_linear(plan, pn, f));
      }

      ++n_report;
      double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - report_wall).count();
      if (el >= 2.0) {
        RCLCPP_INFO(get_logger(), "[rate] stream %6.1f Hz (target %.0f) | plan %s",
                    n_report / el, tick_hz_, have ? "ok" : "WAITING");
        n_report = 0; report_wall = std::chrono::steady_clock::now();
      }

      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      auto tnow = std::chrono::steady_clock::now();
      if (next > tnow) std::this_thread::sleep_until(next);
      else next = tnow;
    }
  }

  double tick_hz_, tick_period_, jtc_horizon_, control_hz_, dt_default_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr plan_sub_;

  std::mutex plan_mtx_;
  std::vector<double> plan_;
  int plan_n_ = 0;
  double plan_dt_ = 0.0, plan_t_ref_ = 0.0;
  bool have_plan_ = false;

  std::thread thread_;
  std::atomic<bool> running_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JtcStreamNode>();
  rclcpp::spin(node);
  node->stop();
  rclcpp::shutdown();
  return 0;
}
