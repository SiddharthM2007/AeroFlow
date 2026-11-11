#include "aeroflow/config.hpp"

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <string_view>

#include "aeroflow/grid3d.hpp"

namespace aeroflow {

namespace {

using nlohmann::json;

[[noreturn]] void fail(const std::string& path, const std::string& message) {
  throw ConfigError(path + ": " + message);
}

const json& require_object(const json& parent, const char* key, const std::string& path) {
  const auto it = parent.find(key);
  if (it == parent.end()) fail(path, "is required");
  if (!it->is_object()) fail(path, "must be an object");
  return *it;
}

void reject_unknown_keys(const json& obj, std::initializer_list<std::string_view> allowed,
                         const std::string& path) {
  for (const auto& [key, _] : obj.items()) {
    bool known = false;
    for (const auto name : allowed) known = known || key == name;
    if (!known) fail(path.empty() ? key : path + "." + key, "unknown field");
  }
}

double require_positive_length(const json& obj, const char* key, const std::string& path) {
  const std::string field = path + "." + key;
  const auto it = obj.find(key);
  if (it == obj.end()) fail(field, "is required");
  if (!it->is_number()) fail(field, "must be a number");
  const double value = it->get<double>();
  if (!std::isfinite(value) || value <= 0.0) fail(field, "must be a finite number > 0");
  return value;
}

std::size_t require_count(const json& obj, const char* key, const std::string& path) {
  const std::string field = path + "." + key;
  const auto it = obj.find(key);
  if (it == obj.end()) fail(field, "is required");
  if (!it->is_number_integer()) fail(field, "must be an integer >= 1");
  if (it->is_number_unsigned()) {
    const auto value = it->get<std::uint64_t>();
    if (value < 1) fail(field, "must be an integer >= 1");
    if (value > Grid3D::kMaxCells) fail(field, "is too large");
    return static_cast<std::size_t>(value);
  }
  if (it->get<std::int64_t>() < 1) fail(field, "must be an integer >= 1");
  return static_cast<std::size_t>(it->get<std::int64_t>());
}

Vec3 require_point(const json& obj, const char* key, const std::string& path) {
  const std::string field = path + "." + key;
  const auto it = obj.find(key);
  if (it == obj.end()) fail(field, "is required");
  if (!it->is_array() || it->size() != 3) fail(field, "must be an array of 3 numbers");
  double v[3];
  for (std::size_t n = 0; n < 3; ++n) {
    const auto& element = (*it)[n];
    if (!element.is_number()) fail(field, "must be an array of 3 numbers");
    v[n] = element.get<double>();
    if (!std::isfinite(v[n])) fail(field, "values must be finite");
  }
  return {v[0], v[1], v[2]};
}

Aabb parse_test_box(const json& obj, const GridConfig& config) {
  const std::string path = "test_box";
  if (!obj.is_object()) fail(path, "must be an object or null");
  reject_unknown_keys(obj, {"min", "max"}, path);
  const Aabb box{require_point(obj, "min", path), require_point(obj, "max", path)};

  const double lo[3] = {box.min.x, box.min.y, box.min.z};
  const double hi[3] = {box.max.x, box.max.y, box.max.z};
  const double len[3] = {config.lx, config.ly, config.lz};
  const char* axis[3] = {"x", "y", "z"};
  for (int a = 0; a < 3; ++a) {
    if (!(lo[a] < hi[a])) {
      fail(path, std::string("min.") + axis[a] + " must be less than max." + axis[a]);
    }
    if (lo[a] < 0.0 || hi[a] > len[a]) {
      fail(path, std::string("extent along ") + axis[a] + " must lie within the domain [0, " +
                     std::to_string(len[a]) + "]");
    }
  }
  return box;
}

}  // namespace

GridConfig parse_config(const json& root) {
  if (!root.is_object()) throw ConfigError("config: top-level value must be a JSON object");
  reject_unknown_keys(root, {"domain", "resolution", "test_box"}, "");

  GridConfig config;

  const json& domain = require_object(root, "domain", "domain");
  reject_unknown_keys(domain, {"lx", "ly", "lz"}, "domain");
  config.lx = require_positive_length(domain, "lx", "domain");
  config.ly = require_positive_length(domain, "ly", "domain");
  config.lz = require_positive_length(domain, "lz", "domain");

  const json& resolution = require_object(root, "resolution", "resolution");
  reject_unknown_keys(resolution, {"nx", "ny", "nz"}, "resolution");
  config.nx = require_count(resolution, "nx", "resolution");
  config.ny = require_count(resolution, "ny", "resolution");
  config.nz = require_count(resolution, "nz", "resolution");

  // Overflow-safe total check (each count is already <= kMaxCells).
  const double total = static_cast<double>(config.nx) * static_cast<double>(config.ny) *
                        static_cast<double>(config.nz);
  if (total > static_cast<double>(Grid3D::kMaxCells)) {
    fail("resolution", "nx*ny*nz must not exceed " + std::to_string(Grid3D::kMaxCells));
  }

  if (const auto it = root.find("test_box"); it != root.end() && !it->is_null()) {
    config.test_box = parse_test_box(*it, config);
  }
  return config;
}

GridConfig parse_config_text(const std::string& text) {
  json root;
  try {
    root = json::parse(text);
  } catch (const json::parse_error& e) {
    throw ConfigError(std::string("config: invalid JSON: ") + e.what());
  }
  return parse_config(root);
}

}  // namespace aeroflow
