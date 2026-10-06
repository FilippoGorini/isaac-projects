// Colour camera focus of the Kinova wrist vision module (Kortex API VisionConfig::DoSensorFocusAction).
// Moves no joint; changes the camera's focus mode. Autofocus comes back after a camera/arm restart, so run it after
// every bringup.
// Usage: vision_focus [robot_ip] manual <value> | disable | continuous | pause | now
//   manual <value>: fixed lens position (reproducible between sessions; intrinsics depend on it)
//   disable: autofocus off (lens position afterwards: to be checked on the robot)
// Built by the vla_kinova_sensors CMakeLists (ported from ros2_kortex_ws/tools/vision_calib).
#include <iostream>
#include <string>
#include <SessionManager.h>
#include <DeviceManagerClientRpc.h>
#include <VisionConfigClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>

namespace k_api = Kinova::Api;
namespace vc = Kinova::Api::VisionConfig;

int main(int argc, char** argv)
{
  int a = 1;
  std::string ip = "192.168.50.12";
  if (argc > 1 && std::string(argv[1]).find('.') != std::string::npos) ip = argv[a++];
  if (argc <= a) { std::cerr << "usage: vision_focus [robot_ip] manual <value> | disable | continuous | pause | now\n"; return 2; }
  const std::string cmd = argv[a];

  vc::SensorFocusAction action;
  action.set_sensor(vc::SENSOR_COLOR);
  if (cmd == "manual" && argc > a + 1) {
    action.set_focus_action(vc::FOCUSACTION_SET_MANUAL_FOCUS);
    action.mutable_manual_focus()->set_value(std::stoul(argv[a + 1]));
  } else if (cmd == "disable") action.set_focus_action(vc::FOCUSACTION_DISABLE_FOCUS);
  else if (cmd == "continuous") action.set_focus_action(vc::FOCUSACTION_START_CONTINUOUS_FOCUS);
  else if (cmd == "pause") action.set_focus_action(vc::FOCUSACTION_PAUSE_CONTINUOUS_FOCUS);
  else if (cmd == "now") action.set_focus_action(vc::FOCUSACTION_FOCUS_NOW);
  else { std::cerr << "unknown command " << cmd << "\n"; return 2; }

  auto transport = new k_api::TransportClientTcp();
  auto router = new k_api::RouterClient(transport, [](k_api::KError e) { std::cerr << "router error: " << e.toString() << std::endl; });
  transport->connect(ip, 10000);
  auto session_info = k_api::Session::CreateSessionInfo();
  session_info.set_username("admin");
  session_info.set_password("admin");
  session_info.set_session_inactivity_timeout(60000);
  session_info.set_connection_inactivity_timeout(2000);
  auto session = new k_api::SessionManager(router);
  int rc = 0;
  try {
    session->CreateSession(session_info);
    uint32_t vision_id = 0;
    const auto devices = k_api::DeviceManager::DeviceManagerClient(router).ReadAllDevices();  // keep it alive for the loop
    for (const auto& h : devices.device_handle())
      if (h.device_type() == k_api::Common::DeviceTypes::VISION) vision_id = h.device_identifier();
    vc::VisionConfigClient(router).DoSensorFocusAction(action, vision_id);
    std::cout << "colour focus: " << cmd << (cmd == "manual" ? std::string(" ") + argv[a + 1] : "") << " (vision device " << vision_id << ")" << std::endl;
    session->CloseSession();
  } catch (k_api::KDetailedException& e) {
    std::cerr << "Kortex error: " << e.what() << std::endl;
    rc = 1;
  }
  router->SetActivationStatus(false);
  transport->disconnect();
  return rc;
}
