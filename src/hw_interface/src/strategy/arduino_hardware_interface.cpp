#include "hw_interface/strategy/arduino_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <sstream>
#include <stdexcept>

namespace hw_interface {

using robot_hw_interface::JointCommand;
using robot_hw_interface::JointState;

namespace {

speed_t toTermiosBaud(int baud_rate) {
    switch (baud_rate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            throw std::invalid_argument("ArduinoHardwareInterface: unsupported baud_rate " +
                                         std::to_string(baud_rate));
    }
}

}  // namespace

ArduinoHardwareInterface::ArduinoHardwareInterface(const std::string& serial_port, int baud_rate,
                                                     int num_joints, int read_timeout_ms)
    : file_descriptor_(-1), num_joints_(num_joints) {
    last_known_state_.position = Eigen::VectorXd::Zero(num_joints_);
    last_known_state_.velocity = Eigen::VectorXd::Zero(num_joints_);
    last_known_state_.torque = Eigen::VectorXd::Zero(num_joints_);
    last_known_state_.current = Eigen::VectorXd::Zero(num_joints_);

    file_descriptor_ = ::open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (file_descriptor_ < 0) {
        throw std::runtime_error("ArduinoHardwareInterface: failed to open serial port '" +
                                  serial_port + "': " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(file_descriptor_, &tty) != 0) {
        ::close(file_descriptor_);
        throw std::runtime_error("ArduinoHardwareInterface: tcgetattr failed on '" + serial_port + "'");
    }

    const speed_t speed = toTermiosBaud(baud_rate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // VMIN=0, VTIME in deciseconds: read() returns after the timeout even
    // if no data has arrived, rather than blocking forever.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = static_cast<cc_t>(read_timeout_ms / 100);

    if (tcsetattr(file_descriptor_, TCSANOW, &tty) != 0) {
        ::close(file_descriptor_);
        throw std::runtime_error("ArduinoHardwareInterface: tcsetattr failed on '" + serial_port + "'");
    }
}

ArduinoHardwareInterface::~ArduinoHardwareInterface() {
    if (file_descriptor_ >= 0) {
        ::close(file_descriptor_);
    }
}

void ArduinoHardwareInterface::write(const JointCommand& command) {
    char mode_char = 'P';
    const Eigen::VectorXd* values = &command.position;
    if (command.control_mode == "velocity") {
        mode_char = 'V';
        values = &command.velocity;
    } else if (command.control_mode == "torque") {
        mode_char = 'T';
        values = &command.torque;
    }

    if (values->size() != num_joints_) {
        throw std::invalid_argument("ArduinoHardwareInterface: command size does not match numJoints()");
    }

    std::ostringstream line;
    line << mode_char;
    for (int i = 0; i < values->size(); ++i) {
        line << ' ' << (*values)[i];
    }
    line << '\n';

    const std::string payload = line.str();
    if (::write(file_descriptor_, payload.data(), payload.size()) < 0) {
        throw std::runtime_error("ArduinoHardwareInterface: serial write failed");
    }
}

std::string ArduinoHardwareInterface::readLine() {
    std::string line;
    char c = 0;
    while (::read(file_descriptor_, &c, 1) == 1) {
        if (c == '\n') {
            return line;
        }
        line += c;
    }
    return line;  // timed out (VTIME) before a full line arrived
}

JointState ArduinoHardwareInterface::read() {
    const std::string line = readLine();
    if (line.empty() || line[0] != 'S') {
        return last_known_state_;  // timeout or malformed line: report last known state
    }

    std::istringstream stream(line.substr(1));
    JointState state;
    state.position.resize(num_joints_);
    state.velocity.resize(num_joints_);
    state.torque.resize(num_joints_);
    state.current.resize(num_joints_);

    for (int i = 0; i < num_joints_; ++i) {
        if (!(stream >> state.position[i])) return last_known_state_;
    }
    for (int i = 0; i < num_joints_; ++i) {
        if (!(stream >> state.velocity[i])) return last_known_state_;
    }
    for (int i = 0; i < num_joints_; ++i) {
        if (!(stream >> state.torque[i])) return last_known_state_;
    }
    for (int i = 0; i < num_joints_; ++i) {
        if (!(stream >> state.current[i])) return last_known_state_;
    }

    last_known_state_ = state;
    return state;
}

int ArduinoHardwareInterface::numJoints() const {
    return num_joints_;
}

}  // namespace hw_interface
