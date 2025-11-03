#pragma once

#include <optional>
#include <sstream>

#include <Eigen/Dense>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>

void configure_logger(const viam::sdk::ResourceConfig &cfg);

// helper function to extract an attribute value from its key within a
// ResourceConfig
template <class T>
std::optional<T> find_config_attribute(const viam::sdk::ResourceConfig &cfg,
                                       const std::string &attribute) {
  auto key = cfg.attributes().find(attribute);
  if (key == cfg.attributes().end()) {
    return std::nullopt;
  }
  const auto *const val = key->second.get<T>();
  if (!val) {
    std::ostringstream buffer;
    buffer << "attribute `" << attribute
           << " could not be converted to the required type";
    throw std::invalid_argument(buffer.str());
  }
  return std::make_optional(*val);
}

Eigen::Vector3d transform_vector(const Eigen::Vector3d &vector,
                                 const Eigen::Matrix3d &rotation_matrix);
