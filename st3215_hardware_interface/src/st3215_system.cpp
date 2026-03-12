#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "st_sc_servo_control_lib/SMS_STS.h"

#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace st3215_hardware_interface
{

class ST3215System : public hardware_interface::SystemInterface
{
public:

hardware_interface::CallbackReturn on_init(
  const hardware_interface::HardwareInfo & info) override
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_name_ = info_.hardware_parameters["port_name"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  size_t n = info_.joints.size();

  position_.resize(n, 0.0);
  velocity_.resize(n, 0.0);
  cmd_position_.resize(n, 0.0);
  cmd_velocity_.resize(n, 0.0);
  last_sent_position_.resize(n, 0.0);
  joint_ids_.resize(n);
  is_velocity_joint_.resize(n, false);

  for (size_t i = 0; i < n; ++i)
  {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));

    for (auto & ci : info_.joints[i].command_interfaces)
    {
      if (ci.name == hardware_interface::HW_IF_VELOCITY)
        is_velocity_joint_[i] = true;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < position_.size(); ++i){
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);

      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      if (is_velocity_joint_[i]) 
      {
        // For wheels: Export ONLY velocity command interface
        command_interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]);
      } 
      else 
      {
        // For arm joints: Export ONLY position command interface
        command_interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
      }
    }

    return command_interfaces;
  }


hardware_interface::CallbackReturn on_activate(
  const rclcpp_lifecycle::State &) override
{
  std::cout << "--- STARTING ON_ACTIVATE ---" << std::endl;

  if (!servo_.begin(baud_rate_, port_name_.c_str()))
  {
    std::cout << "FAILED TO OPEN SERIAL PORT" << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::cout << "Serial port opened successfully." << std::endl;

  for (size_t i = 0; i < joint_ids_.size(); ++i)
  {
    int id = joint_ids_[i];

    servo_.EnableTorque(id, 0);

    if (is_velocity_joint_[i])
    {
      // ================= WHEELS =================
      servo_.WheelMode(id);
      servo_.EnableTorque(id, 1);
      servo_.WriteSpe(id, 0, 0);

      cmd_velocity_[i] = 0.0;
      velocity_[i] = 0.0;
    }
    else
    {
      // ================= ARM =================
      servo_.EnableTorque(id, 1);

      // Read current physical position
      int raw_pos = servo_.ReadPos(id);

     // if (raw_pos == -1)
      //{
        //std::cout << "Failed to read position for ID: " << id << std::endl;
       // return hardware_interface::CallbackReturn::ERROR;
      //}

      // double rad = raw_pos * (2.0 * M_PI / 4096.0);  // pi values ka hai
      double rad =  joint_directions_[i]*(raw_pos - 2048.0) * (2.0 * M_PI / 4096.0);
      // IMPORTANT: Sync everything to current position
      position_[i] = rad;
      cmd_position_[i] = rad;
      last_sent_position_[i] = rad;
    }
  }

  std::cout << "--- ON_ACTIVATE COMPLETE ---" << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    for (auto id : joint_ids_)
    {
      servo_.EnableTorque(id, 0);
    }

    servo_.end();

    RCLCPP_INFO(rclcpp::get_logger("ST3215System"),
                "ST3215 hardware deactivated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }
hardware_interface::return_type read(
  const rclcpp::Time &, const rclcpp::Duration &) override
{
  // Completely disable all hardware reads.
  // Just mirror commands as feedback.

  for (size_t i = 0; i < joint_ids_.size(); ++i)
  {
    if (is_velocity_joint_[i])
    {
      velocity_[i] = cmd_velocity_[i];
    }
    else
    {
      position_[i] = cmd_position_[i];
      velocity_[i] = 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type write(
  const rclcpp::Time &, const rclcpp::Duration &) override
{
  for (size_t i = 0; i < joint_ids_.size(); ++i)
  {
    int id = joint_ids_[i];

    if (is_velocity_joint_[i])
    {
      // Clamp velocity to safe range
      double vel = std::clamp(cmd_velocity_[i], -2.0, 2.0);

      int raw_speed = static_cast<int>(vel * 200.0);

      // Skip tiny values to reduce bus spam
      if (std::abs(raw_speed) < 1)
        raw_speed = 0;

      // Fire and forget (ignore return)
      servo_.WriteSpe(id, raw_speed, 0);
    }
    else
    {
      if (std::abs(cmd_position_[i] - last_sent_position_[i]) > 0.002)
      {
        double pos = std::clamp(cmd_position_[i], -3.14, 3.14);

        int raw_pos = static_cast<int>(
          (joint_directions_[i]*pos * (4096.0 / (2.0 * M_PI))) + 2048.0
        );

        servo_.WritePosEx(id, raw_pos, 200, 20);

        last_sent_position_[i] = cmd_position_[i];
      }
    }
  }

  return hardware_interface::return_type::OK;
}

private:
  SMS_STS servo_;

  std::string port_name_;
  int baud_rate_;

  std::vector<int> joint_ids_;
  int read_counter_ = 0;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> cmd_position_;
  std::vector<double> cmd_velocity_;
  std::vector<double> last_sent_position_;
  std::vector<bool> is_velocity_joint_;
  std::vector<int> joint_directions_ =
{ 1,1,1,
  1,
  -1,
  -1,
  -1,
  -1,
  1
};
rclcpp::Clock clock_{RCL_ROS_TIME};
};

}  // namespace st3215_hardware_interface


PLUGINLIB_EXPORT_CLASS(
  st3215_hardware_interface::ST3215System,
  hardware_interface::SystemInterface)


  
