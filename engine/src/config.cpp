#include "aeroflow/config.hpp"

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <string_view>

#include <algorithm>

#include "aeroflow/field_export.hpp"
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

MeshSource parse_mesh(const json& obj) {
  const std::string path = "mesh";
  if (!obj.is_object()) fail(path, "must be an object or null");
  reject_unknown_keys(obj, {"triangles_file", "bvh_leaf_size"}, path);
  MeshSource mesh;
  const auto file = obj.find("triangles_file");
  if (file == obj.end()) fail(path + ".triangles_file", "is required");
  if (!file->is_string() || file->get<std::string>().empty()) {
    fail(path + ".triangles_file", "must be a non-empty string");
  }
  mesh.triangles_file = file->get<std::string>();
  if (obj.contains("bvh_leaf_size")) {
    mesh.bvh_leaf_size = require_count(obj, "bvh_leaf_size", path);
    if (mesh.bvh_leaf_size > 64) fail(path + ".bvh_leaf_size", "must be <= 64");
  }
  return mesh;
}

FieldsRequest parse_fields(const json& obj) {
  const std::string path = "fields";
  if (!obj.is_object()) fail(path, "must be an object or null");
  reject_unknown_keys(obj, {"output_dir", "requested"}, path);
  FieldsRequest fields;
  const auto dir = obj.find("output_dir");
  if (dir == obj.end() || !dir->is_string() || dir->get<std::string>().empty()) {
    fail(path + ".output_dir", "must be a non-empty string");
  }
  fields.output_dir = dir->get<std::string>();
  const auto requested = obj.find("requested");
  if (requested == obj.end() || !requested->is_array()) fail(path + ".requested", "must be an array of field ids");
  const auto known = known_field_ids();
  for (const auto& id : *requested) {
    if (!id.is_string()) fail(path + ".requested", "must contain only strings");
    const auto name = id.get<std::string>();
    if (std::find(known.begin(), known.end(), name) == known.end()) {
      fail(path + ".requested", "unknown field id '" + name + "'");
    }
    if (std::find(fields.requested.begin(), fields.requested.end(), name) != fields.requested.end()) {
      fail(path + ".requested", "duplicate field id '" + name + "'");
    }
    fields.requested.push_back(name);
  }
  return fields;
}

double optional_number(const json& obj, const char* key, const std::string& path, double fallback,
                       bool (*ok)(double), const char* requirement) {
  const auto it = obj.find(key);
  if (it == obj.end()) return fallback;
  const std::string field = path + "." + key;
  if (!it->is_number()) fail(field, std::string("must be a number ") + requirement);
  const double v = it->get<double>();
  if (!std::isfinite(v) || !ok(v)) fail(field, std::string("must be ") + requirement);
  return v;
}

PressureSolverKind parse_solver(const json& obj, const std::string& path) {
  const auto it = obj.find("pressure_solver");
  if (it == obj.end() || it->is_null()) return PressureSolverKind::Auto;
  if (it->is_string()) {
    if (const auto kind = parse_pressure_solver(it->get<std::string>())) return *kind;
  }
  fail(path + ".pressure_solver", "must be \"auto\", \"mgpcg\" or \"jacobi_pcg\"");
}

ProjectionConfig parse_projection(const json& obj) {
  const std::string path = "projection";
  if (!obj.is_object()) fail(path, "must be an object or null");
  reject_unknown_keys(obj, {"inlet_velocity", "density", "dt", "tolerance", "max_iterations", "pressure_solver"}, path);
  ProjectionConfig p;
  p.pressure_solver = parse_solver(obj, path);
  p.inlet_velocity = optional_number(obj, "inlet_velocity", path, p.inlet_velocity,
                                     [](double v) { return v >= 0.0; }, "finite and >= 0 (m/s)");
  p.density = optional_number(obj, "density", path, p.density, [](double v) { return v > 0.0; },
                              "finite and > 0 (kg/m^3)");
  if (const auto it = obj.find("dt"); it != obj.end() && !it->is_null()) {
    p.dt = optional_number(obj, "dt", path, 0.0, [](double v) { return v > 0.0; }, "finite and > 0 (s)");
  }
  p.tolerance = optional_number(obj, "tolerance", path, p.tolerance,
                                [](double v) { return v > 0.0 && v < 1.0; }, "in (0, 1)");
  if (obj.contains("max_iterations")) {
    p.max_iterations = require_count(obj, "max_iterations", path);
    if (p.max_iterations > 1000000) fail(path + ".max_iterations", "must be <= 1000000");
  }
  return p;
}

SimulationConfig parse_simulation(const json& obj) {
  const std::string path = "simulation";
  if (!obj.is_object()) fail(path, "must be an object or null");
  reject_unknown_keys(obj,
                      {"inlet_velocity", "density", "viscosity", "duration", "frame_interval", "cfl", "dt",
                       "max_steps", "tolerance", "max_iterations", "initial", "max_wall_seconds", "pressure_solver"},
                      path);
  SimulationConfig s;
  s.inlet_velocity = optional_number(obj, "inlet_velocity", path, s.inlet_velocity,
                                     [](double v) { return v >= 0.0; }, "finite and >= 0 (m/s)");
  s.density = optional_number(obj, "density", path, s.density, [](double v) { return v > 0.0; },
                              "finite and > 0 (kg/m^3)");
  s.viscosity = optional_number(obj, "viscosity", path, s.viscosity, [](double v) { return v >= 0.0; },
                                "finite and >= 0 (m^2/s)");
  s.duration = optional_number(obj, "duration", path, s.duration, [](double v) { return v > 0.0; },
                               "finite and > 0 (s)");
  s.frame_interval = optional_number(obj, "frame_interval", path, s.frame_interval,
                                     [](double v) { return v > 0.0; }, "finite and > 0 (s)");
  s.cfl = optional_number(obj, "cfl", path, s.cfl, [](double v) { return v > 0.0 && v <= 1.0; }, "in (0, 1]");
  if (const auto it = obj.find("dt"); it != obj.end() && !it->is_null()) {
    s.dt = optional_number(obj, "dt", path, 0.0, [](double v) { return v > 0.0; }, "finite and > 0 (s)");
  }
  s.tolerance = optional_number(obj, "tolerance", path, s.tolerance,
                                [](double v) { return v > 0.0 && v < 1.0; }, "in (0, 1)");
  s.max_wall_seconds = optional_number(obj, "max_wall_seconds", path, s.max_wall_seconds,
                                       [](double v) { return v > 0.0 && v <= 86400.0; }, "in (0, 86400] s");
  if (obj.contains("max_steps")) {
    s.max_steps = require_count(obj, "max_steps", path);
    if (s.max_steps > 1000000) fail(path + ".max_steps", "must be <= 1000000");
  }
  if (obj.contains("max_iterations")) {
    s.max_iterations = require_count(obj, "max_iterations", path);
    if (s.max_iterations > 1000000) fail(path + ".max_iterations", "must be <= 1000000");
  }
  if (const auto it = obj.find("initial"); it != obj.end()) {
    if (!it->is_string() || (*it != "uniform" && *it != "rest")) {
      fail(path + ".initial", "must be \"uniform\" or \"rest\"");
    }
    s.initial = it->get<std::string>();
  }
  if (s.duration / s.frame_interval > 10000.0) fail(path, "duration / frame_interval must not exceed 10000 frames");
  s.pressure_solver = parse_solver(obj, path);
  return s;
}

}  // namespace

GridConfig parse_config(const json& root) {
  if (!root.is_object()) throw ConfigError("config: top-level value must be a JSON object");
  reject_unknown_keys(root, {"domain", "resolution", "test_box", "mesh", "fields", "projection", "simulation"}, "");

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
  if (const auto it = root.find("mesh"); it != root.end() && !it->is_null()) {
    if (config.test_box) {
      fail("mesh", "test_box and mesh are mutually exclusive; choose one geometry source");
    }
    config.mesh = parse_mesh(*it);
  }
  if (const auto it = root.find("fields"); it != root.end() && !it->is_null()) {
    config.fields = parse_fields(*it);
  }
  if (const auto it = root.find("projection"); it != root.end() && !it->is_null()) {
    config.projection = parse_projection(*it);
  }
  if (const auto it = root.find("simulation"); it != root.end() && !it->is_null()) {
    config.simulation = parse_simulation(*it);
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
