#include "kinematics/factory/kinematics_factory.hpp"

#include <array>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "kinematics/strategy/analytical_six_axis_forward_kinematics.hpp"
#include "kinematics/strategy/analytical_six_axis_inverse_kinematics.hpp"
#include "kinematics/strategy/kdl_forward_kinematics.hpp"
#include "kinematics/strategy/kdl_inverse_kinematics.hpp"

namespace kinematics {

namespace {

SixAxisLinkLengths parseSixAxisLinkLengths(const YAML::Node& kinematics_params) {
    const YAML::Node lengths_node = kinematics_params["link_lengths"];
    if (!lengths_node) {
        throw std::runtime_error("kinematics_factory: 'analytical' method requires a 'link_lengths' map");
    }
    SixAxisLinkLengths lengths;
    lengths.l1 = lengths_node["l1"].as<double>();
    lengths.d2 = lengths_node["d2"].as<double>();
    lengths.l3 = lengths_node["l3"].as<double>();
    lengths.l5 = lengths_node["l5"].as<double>();
    lengths.l6 = lengths_node["l6"].as<double>();
    lengths.l7 = lengths_node["l7"].as<double>();
    lengths.le = lengths_node["le"].as<double>();
    return lengths;
}

std::shared_ptr<IFKStrategy> createAnalyticalFKStrategy(const YAML::Node& kinematics_params) {
    return std::make_shared<AnalyticalSixAxisForwardKinematics>(parseSixAxisLinkLengths(kinematics_params));
}

// Runs `xacro <path>` and captures stdout. The path is expected to come
// from trusted local config (robot_config yaml), not untrusted/remote
// input; it is still single-quote-escaped before being handed to the shell.
std::string runXacro(const std::string& xacro_path) {
    std::string escaped;
    for (char c : xacro_path) {
        if (c == '\'') {
            escaped += "'\\''";
        } else {
            escaped += c;
        }
    }
    const std::string command = "xacro '" + escaped + "'";

    std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("kinematics_factory: failed to run xacro on '" + xacro_path + "'");
    }
    std::string output;
    std::array<char, 4096> buffer{};
    std::size_t bytes_read = 0;
    while ((bytes_read = fread(buffer.data(), 1, buffer.size(), pipe.get())) > 0) {
        output.append(buffer.data(), bytes_read);
    }
    return output;
}

// Resolves "package://<pkg>/<rel_path>" against that package's installed
// share directory; any other string is returned unchanged (a plain
// filesystem path).
std::string resolvePackagePath(const std::string& path) {
    constexpr std::string_view kPrefix = "package://";
    if (path.compare(0, kPrefix.size(), kPrefix) != 0) {
        return path;
    }
    const std::string rest = path.substr(kPrefix.size());
    const std::size_t slash_pos = rest.find('/');
    if (slash_pos == std::string::npos) {
        throw std::runtime_error("kinematics_factory: malformed package:// path '" + path + "'");
    }
    const std::string package_name = rest.substr(0, slash_pos);
    const std::string relative_path = rest.substr(slash_pos + 1);
    return ament_index_cpp::get_package_share_directory(package_name) + "/" + relative_path;
}

std::string loadUrdfXml(const std::string& urdf_path) {
    const std::string resolved_path = resolvePackagePath(urdf_path);

    constexpr std::string_view kXacroExtension = ".xacro";
    if (resolved_path.size() >= kXacroExtension.size() &&
        resolved_path.compare(resolved_path.size() - kXacroExtension.size(), kXacroExtension.size(),
                               kXacroExtension) == 0) {
        return runXacro(resolved_path);
    }

    std::ifstream file(resolved_path);
    if (!file) {
        throw std::runtime_error("kinematics_factory: cannot open URDF file '" + resolved_path + "'");
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::shared_ptr<IFKStrategy> createKdlFKStrategy(const YAML::Node& kinematics_params) {
    if (!kinematics_params["urdf_path"]) {
        throw std::runtime_error("kinematics_factory: 'kdl' method requires a 'urdf_path'");
    }
    const std::string urdf_path = kinematics_params["urdf_path"].as<std::string>();
    const std::string start_link = kinematics_params["start_link"].as<std::string>("base_link");
    const std::string end_link = kinematics_params["end_link"].as<std::string>("ee_link");
    return std::make_shared<KdlForwardKinematics>(loadUrdfXml(urdf_path), start_link, end_link);
}

}  // namespace

std::shared_ptr<IFKStrategy> createFKStrategy(const std::string& method,
                                               const YAML::Node& kinematics_params) {
    if (method == "analytical") {
        return createAnalyticalFKStrategy(kinematics_params);
    }
    if (method == "kdl") {
        return createKdlFKStrategy(kinematics_params);
    }
    if (method == "poe" || method == "pinocchio") {
        throw std::runtime_error("kinematics_factory: method '" + method +
                                  "' is not implemented yet");
    }
    throw std::runtime_error("kinematics_factory: unknown kinematics_method '" + method + "'");
}

std::unique_ptr<IIKStrategy> createIKStrategy(const std::string& method,
                                               std::shared_ptr<IFKStrategy> fk_strategy) {
    if (method == "analytical") {
        auto analytical_fk = std::dynamic_pointer_cast<AnalyticalSixAxisForwardKinematics>(fk_strategy);
        if (!analytical_fk) {
            throw std::runtime_error(
                "kinematics_factory: ik_method 'analytical' requires kinematics_method 'analytical'");
        }
        return std::make_unique<AnalyticalSixAxisInverseKinematics>(std::move(analytical_fk));
    }
    if (method == "kdl") {
        auto kdl_fk = std::dynamic_pointer_cast<KdlForwardKinematics>(fk_strategy);
        if (!kdl_fk) {
            throw std::runtime_error("kinematics_factory: ik_method 'kdl' requires kinematics_method 'kdl'");
        }
        return std::make_unique<KdlInverseKinematics>(std::move(kdl_fk));
    }
    throw std::runtime_error("kinematics_factory: unknown ik_method '" + method + "'");
}

}  // namespace kinematics
