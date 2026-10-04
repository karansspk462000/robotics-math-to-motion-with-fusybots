#include "hw_interface/factory/hw_interface_factory.hpp"

#include <stdexcept>

#include "hw_interface/strategy/arduino_hardware_interface.hpp"
#include "hw_interface/strategy/dummy_hardware_interface.hpp"

namespace hw_interface {

std::unique_ptr<robot_hw_interface::IRobotInterface> createHardwareInterface(
    const std::string& method, int num_joints, const YAML::Node& params,
    const std::vector<double>& home_joint_position) {
    if (method == "dummy") {
        return std::make_unique<DummyHardwareInterface>(num_joints, home_joint_position);
    }
    if (method == "arduino") {
        if (!params["serial_port"]) {
            throw std::runtime_error("hw_interface_factory: 'arduino' method requires a 'serial_port'");
        }
        const std::string serial_port = params["serial_port"].as<std::string>();
        const int baud_rate = params["baud_rate"].as<int>(115200);
        const int read_timeout_ms = params["read_timeout_ms"].as<int>(100);
        return std::make_unique<ArduinoHardwareInterface>(serial_port, baud_rate, num_joints,
                                                            read_timeout_ms);
    }
    throw std::runtime_error("hw_interface_factory: unknown hardware_method '" + method + "'");
}

}  // namespace hw_interface
