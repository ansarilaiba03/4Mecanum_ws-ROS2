#include "mecanum_firmware/mecanum_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace mecanum_firmware
{

static std::string formatWheelCmd(const std::string &prefix, double value)
{
  char sign = value >= 0 ? 'p' : 'n';
  double abs_val = std::abs(value);
  std::ostringstream oss;
  oss << prefix << sign
      << std::fixed << std::setprecision(2) << abs_val;
  return oss.str();
}

MecanumInterface::MecanumInterface() {}

MecanumInterface::~MecanumInterface()
{
  if (arduino_.IsOpen())
  {
    try { arduino_.Close(); }
    catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("MecanumInterface"),
                          "Error closing port " << port_);
    }
  }
}

CallbackReturn MecanumInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) return result;

  try { port_ = info_.hardware_parameters.at("port"); }
  catch (const std::out_of_range &) {
    RCLCPP_FATAL(rclcpp::get_logger("MecanumInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  if (info_.joints.size() != 4) {
    RCLCPP_FATAL(rclcpp::get_logger("MecanumInterface"),
                 "Expected 4 joints, got %zu. Aborting.", info_.joints.size());
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.assign(4, 0.0);
  position_states_.assign(4, 0.0);
  velocity_states_.assign(4, 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
  }
  return command_interfaces;
}

CallbackReturn MecanumInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumInterface"), "Starting robot hardware ...");

  velocity_commands_.assign(4, 0.0);
  position_states_.assign(4, 0.0);
  velocity_states_.assign(4, 0.0);

  try {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    // Give Arduino time to reset after serial connection opens
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  catch (...) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("MecanumInterface"),
                        "Could not open port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("MecanumInterface"), "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumInterface"), "Stopping robot hardware ...");
  if (arduino_.IsOpen()) {
    try { arduino_.Close(); }
    catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("MecanumInterface"),
                          "Error closing port " << port_);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("MecanumInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumInterface::read(const rclcpp::Time &,
                                                        const rclcpp::Duration &)
{
  // Open-loop: echo commands as states until encoders are added
  for (size_t i = 0; i < 4; i++)
    velocity_states_[i] = velocity_commands_[i];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumInterface::write(const rclcpp::Time &,
                                                         const rclcpp::Duration &)
{
  // Format: "frp66.67,flp66.67,rrp66.67,rlp66.67,\n"
  // No zero-padding — Arduino's toFloat() handles it fine without it
  std::string message =
      formatWheelCmd("fr", velocity_commands_[0]) + "," +
      formatWheelCmd("fl", velocity_commands_[1]) + "," +
      formatWheelCmd("rr", velocity_commands_[2]) + "," +
      formatWheelCmd("rl", velocity_commands_[3]) + ",\n";

  try {
    arduino_.Write(message);
    RCLCPP_INFO(rclcpp::get_logger("MecanumInterface"), "Sending: %s", message.c_str());
    // No DrainWriteBuffer() — it blocks and causes the Arduino safety timeout to fire
  }
  catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MecanumInterface"),
                        "Write failed for message: " << message);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

} // namespace mecanum_firmware

PLUGINLIB_EXPORT_CLASS(mecanum_firmware::MecanumInterface, hardware_interface::SystemInterface)