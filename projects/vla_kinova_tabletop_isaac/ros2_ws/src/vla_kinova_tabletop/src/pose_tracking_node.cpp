// Thin wrapper around moveit_servo::PoseTracking. Subscribes (via the
// PoseTracking class) to "target_pose" (geometry_msgs/PoseStamped) and drives
// the end-effector toward each new target via Servo. Modelled on the stock
// pose_tracking_demo, but without the hardcoded waypoint loop.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>

#include <chrono>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("kinova_pose_tracking_node");

class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(
        topic, rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) { statusCB(msg); });
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
  {
    moveit_servo::StatusCode latest = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest != status_)
    {
      status_ = latest;
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << moveit_servo::SERVO_STATUS_CODE_MAP.at(status_));
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pose_tracking_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  // RAII: every early `return EXIT_FAILURE` below would otherwise destroy
  // `executor_thread` while still joinable, which is std::terminate → SIGABRT
  // ("terminate called without an active exception", exit -6) and masks the
  // real error in the logs.
  struct ExecutorThreadJoiner
  {
    rclcpp::executors::SingleThreadedExecutor& executor;
    std::thread& thread;
    ~ExecutorThreadJoiner()
    {
      executor.cancel();
      if (thread.joinable())
        thread.join();
    }
  } executor_joiner{ executor, executor_thread };

  // When use_sim_time=true, node->now() returns 0 until the first /clock
  // message reaches this node. PlanningSceneMonitor::waitForCurrentRobotState
  // short-circuits to false on t==0, so passing node->now() while the clock
  // is still 0 fails instantly with a misleading "Timed out" — the 30 s never
  // actually starts. Block on a wall-clock deadline until /clock has ticked.
  if (node->get_parameter("use_sim_time").as_bool())
  {
    RCLCPP_INFO(LOGGER, "use_sim_time=true; waiting for /clock to start ticking...");
    const auto wall_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (rclcpp::ok() && node->now().nanoseconds() == 0 &&
           std::chrono::steady_clock::now() < wall_deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (node->now().nanoseconds() == 0)
    {
      RCLCPP_FATAL(LOGGER, "No /clock received within 30s. Is Isaac Sim running and Playing?");
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
    RCLCPP_INFO(LOGGER, "Clock is ticking (sim t=%.3fs)", node->now().seconds());
  }

  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Could not load Servo parameters. Check that the 'moveit_servo.*' params are set.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Planning scene monitor — required by Servo for collision checking and TF.
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!psm->getPlanningScene())
  {
    RCLCPP_FATAL(LOGGER, "Failed to set up PlanningSceneMonitor.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  psm->providePlanningSceneService();
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  psm->startStateMonitor(servo_parameters->joint_topic);
  psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  if (!psm->waitForCurrentRobotState(node->now() - rclcpp::Duration::from_seconds(0.1), 30.0))
  {
    RCLCPP_FATAL(LOGGER, "Timed out waiting for first robot state. Is joint_state_broadcaster running?");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // DEBUG: log PSM's direct EE pose query so we can compare to what PoseTracking
  // sees. If PSM returns the homed EE pose but PoseTracking later returns
  // identity, the bug is inside moveit_servo, not in the input state.
  {
    auto state_monitor = psm->getStateMonitor();
    auto current_state = state_monitor ? state_monitor->getCurrentState() : nullptr;
    if (!current_state)
    {
      RCLCPP_WARN(LOGGER, "[DEBUG] PSM has NO current state at PoseTracking-construction time!");
    }
    else
    {
      const Eigen::Isometry3d& ee = current_state->getGlobalLinkTransform(servo_parameters->ee_frame_name);
      const auto& t = ee.translation();
      Eigen::Quaterniond q(ee.rotation());
      RCLCPP_INFO(LOGGER,
                  "[DEBUG] PSM EE (%s in %s): pos=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
                  servo_parameters->ee_frame_name.c_str(), servo_parameters->planning_frame.c_str(),
                  t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    }
  }

  // The PoseTracking class internally subscribes to "target_pose" (PoseStamped).
  moveit_servo::PoseTracking tracker(node, servo_parameters, psm);

  // DEBUG: PoseTracking's view of the EE pose (used as `current` in error calc).
  // This should match the PSM log above. If it's identity / zeros, that's the bug.
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    if (tracker.getCommandFrameTransform(tf_msg))
    {
      const auto& tr = tf_msg.transform.translation;
      const auto& q = tf_msg.transform.rotation;
      RCLCPP_INFO(LOGGER,
                  "[DEBUG] tracker.getCommandFrameTransform OK: parent=%s child=%s pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(),
                  tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w);
    }
    else
    {
      RCLCPP_WARN(LOGGER, "[DEBUG] tracker.getCommandFrameTransform returned false — this is the bug!");
    }
  }

  // Log Servo's status whenever it changes.
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  // Tolerances are kept tight so that tracking is "continuous": Servo never
  // declares the target reached and exits — it keeps chasing new targets.
  const Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };
  const double rot_tol = 0.01;
  const double target_pose_timeout = 0.5;  // seconds since last target before bailing out

  RCLCPP_INFO(LOGGER, "Pose tracking ready. Publish geometry_msgs/PoseStamped on 'target_pose' to drive the EE.");

  // moveToPose() blocks. Wrap it in a loop so the node keeps running even
  // when no target_pose is being published (e.g., between teleop sessions).
  while (rclcpp::ok())
  {
    auto status = tracker.moveToPose(lin_tol, rot_tol, target_pose_timeout);
    RCLCPP_INFO_STREAM(LOGGER, "PoseTracking iteration ended: "
                                   << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(status));

    // DEBUG: per-iteration snapshot of the EE pose PoseTracking knows about and
    // the one PSM directly reports. Either of these going stale -> error
    // computation breaks.
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      if (tracker.getCommandFrameTransform(tf_msg))
      {
        const auto& tr = tf_msg.transform.translation;
        const auto& q = tf_msg.transform.rotation;
        RCLCPP_INFO(LOGGER, "[DEBUG] tracker EE pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
                    tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w);
      }
      else
      {
        RCLCPP_WARN(LOGGER, "[DEBUG] tracker.getCommandFrameTransform returned false");
      }
      auto state_monitor = psm->getStateMonitor();
      auto current_state = state_monitor ? state_monitor->getCurrentState() : nullptr;
      if (current_state)
      {
        const Eigen::Isometry3d& ee = current_state->getGlobalLinkTransform(servo_parameters->ee_frame_name);
        const auto& t = ee.translation();
        RCLCPP_INFO(LOGGER, "[DEBUG] PSM EE pos=(%.3f, %.3f, %.3f)", t.x(), t.y(), t.z());
      }
    }

    tracker.resetTargetPose();
  }

  // executor_joiner (RAII) handles executor.cancel() + executor_thread.join().
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
