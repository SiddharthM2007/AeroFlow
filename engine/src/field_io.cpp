#include "aeroflow/field_io.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>

namespace aeroflow {

namespace {

static_assert(std::endian::native == std::endian::little, "AFFIELD I/O assumes a little-endian host");

constexpr std::size_t kPrefixBytes = 16;
constexpr std::size_t kAlign = 8;

std::size_t align_up(std::size_t n) { return (n + kAlign - 1) / kAlign * kAlign; }

std::size_t element_count(const std::array<std::size_t, 3>& shape) {
  std::size_t n = 1;
  for (std::size_t e : shape) {
    if (e == 0 || n > std::numeric_limits<std::size_t>::max() / e) throw FieldError("invalid field shape");
    n *= e;
  }
  return n;
}

nlohmann::json stats_json(const FieldStats& s) {
  if (s.valid_count == 0) {
    return {{"valid_count", 0}, {"min", nullptr}, {"max", nullptr}, {"max_abs", nullptr},
            {"mean_abs", nullptr}, {"rms", nullptr}};
  }
  return {{"valid_count", s.valid_count}, {"min", s.min},           {"max", s.max},
          {"max_abs", s.max_abs},         {"mean_abs", s.mean_abs}, {"rms", s.rms}};
}

}  // namespace

const char* to_string(FieldLocation location) noexcept {
  switch (location) {
    case FieldLocation::Cell: return "cell";
    case FieldLocation::FaceX: return "face_x";
    case FieldLocation::FaceY: return "face_y";
    case FieldLocation::FaceZ: return "face_z";
  }
  return "unknown";
}

std::array<std::size_t, 3> field_shape(FieldLocation location, std::size_t nx, std::size_t ny, std::size_t nz) {
  switch (location) {
    case FieldLocation::FaceX: return {nx + 1, ny, nz};
    case FieldLocation::FaceY: return {nx, ny + 1, nz};
    case FieldLocation::FaceZ: return {nx, ny, nz + 1};
    case FieldLocation::Cell:
    default: return {nx, ny, nz};
  }
}

FieldStats compute_field_stats(std::span<const double> values, std::span<const std::uint8_t> valid) {
  if (values.size() != valid.size()) throw FieldError("values and valid blocks differ in length");
  FieldStats s;
  double sum_abs = 0.0, sum_sq = 0.0;
  s.min = std::numeric_limits<double>::infinity();
  s.max = -std::numeric_limits<double>::infinity();
  for (std::size_t n = 0; n < values.size(); ++n) {
    if (!valid[n]) continue;
    const double v = values[n];
    const double a = std::abs(v);
    s.min = std::min(s.min, v);
    s.max = std::max(s.max, v);
    s.max_abs = std::max(s.max_abs, a);
    sum_abs += a;
    sum_sq += v * v;
    ++s.valid_count;
  }
  if (s.valid_count == 0) return FieldStats{};
  const double count = static_cast<double>(s.valid_count);
  s.mean_abs = sum_abs / count;
  s.rms = std::sqrt(sum_sq / count);
  return s;
}

nlohmann::json field_header(const ScalarField& field, const MacGrid& grid, const FieldStats& stats) {
  const auto expected = field_shape(field.location, grid.nx(), grid.ny(), grid.nz());
  if (field.shape != expected) throw FieldError("field shape does not match its location on the grid");
  return {
      {"format", "aeroflow-field"},
      {"version", kFieldFormatVersion},
      {"name", field.name},
      {"quantity", field.quantity},
      {"long_name", field.long_name},
      {"units", field.units},
      {"signed", field.is_signed},
      {"kind", "scalar"},
      {"components", 1},
      {"location", to_string(field.location)},
      {"shape", field.shape},
      {"index_order", "i + n0*(j + n1*k)"},
      {"byte_order", "little"},
      {"grid",
       {{"resolution", {{"nx", grid.nx()}, {"ny", grid.ny()}, {"nz", grid.nz()}}},
        {"origin", {0.0, 0.0, 0.0}},
        {"spacing", {grid.dx(), grid.dy(), grid.dz()}},
        {"domain", {grid.lx(), grid.ly(), grid.lz()}}}},
      {"frame", field.frame.is_null() ? nlohmann::json{{"index", 0}, {"time_s", nullptr}, {"label", ""}} : field.frame},
      {"mask", field.mask},
      {"provenance", field.provenance},
      {"stats", stats_json(stats)},
  };
}

nlohmann::json write_field_file(const std::string& path, const ScalarField& field, const MacGrid& grid) {
  const std::size_t n = element_count(field.shape);
  if (field.values.size() != n || field.valid.size() != n) {
    throw FieldError("field '" + field.name + "' has " + std::to_string(field.values.size()) + " values and " +
                     std::to_string(field.valid.size()) + " validity flags; shape requires " + std::to_string(n));
  }
  const FieldStats stats = compute_field_stats(field.values, field.valid);
  nlohmann::json header = field_header(field, grid, stats);

  // Block offsets depend on the header length, which depends on the offsets' digits: iterate
  // until the padded header length is stable (converges in one or two passes).
  std::string text;
  std::size_t header_len = 0;
  for (int pass = 0; pass < 4; ++pass) {
    const std::size_t values_offset = kPrefixBytes + header_len;
    const std::size_t values_len = n * sizeof(float);
    const std::size_t valid_offset = align_up(values_offset + values_len);
    header["blocks"] = {
        {{"name", "values"}, {"dtype", "float32"}, {"offset", values_offset}, {"length", values_len}},
        {{"name", "valid"}, {"dtype", "uint8"}, {"offset", valid_offset}, {"length", n}},
    };
    text = header.dump();
    const std::size_t padded = align_up(kPrefixBytes + text.size()) - kPrefixBytes;
    if (padded == header_len) break;
    header_len = padded;
  }
  if (text.size() > header_len) throw FieldError("AFFIELD header length did not converge");
  text.resize(header_len, ' ');

  std::vector<float> narrowed(n);
  for (std::size_t i = 0; i < n; ++i) narrowed[i] = static_cast<float>(field.values[i]);

  std::ofstream out(path, std::ios::binary);
  if (!out) throw FieldError("cannot write field file: " + path);
  const std::uint32_t version = kFieldFormatVersion;
  const auto len32 = static_cast<std::uint32_t>(header_len);
  out.write(kFieldMagic, sizeof(kFieldMagic));
  out.write(reinterpret_cast<const char*>(&version), sizeof(version));
  out.write(reinterpret_cast<const char*>(&len32), sizeof(len32));
  out.write(text.data(), static_cast<std::streamsize>(text.size()));
  out.write(reinterpret_cast<const char*>(narrowed.data()), static_cast<std::streamsize>(n * sizeof(float)));
  const std::size_t written = kPrefixBytes + header_len + n * sizeof(float);
  const std::string pad(align_up(written) - written, '\0');
  out.write(pad.data(), static_cast<std::streamsize>(pad.size()));
  out.write(reinterpret_cast<const char*>(field.valid.data()), static_cast<std::streamsize>(n));
  if (!out) throw FieldError("failed to write field file: " + path);
  return header;
}

FieldFile read_field_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw FieldError("cannot open field file: " + path);
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (bytes.size() < kPrefixBytes || std::memcmp(bytes.data(), kFieldMagic, sizeof(kFieldMagic)) != 0) {
    throw FieldError("not an AFFIELD file");
  }
  std::uint32_t version = 0, header_len = 0;
  std::memcpy(&version, bytes.data() + 8, 4);
  std::memcpy(&header_len, bytes.data() + 12, 4);
  if (version != kFieldFormatVersion) throw FieldError("unsupported AFFIELD version " + std::to_string(version));
  if (kPrefixBytes + header_len > bytes.size()) throw FieldError("AFFIELD header is truncated");

  FieldFile f;
  try {
    f.header = nlohmann::json::parse(bytes.substr(kPrefixBytes, header_len));
  } catch (const nlohmann::json::exception& e) {
    throw FieldError(std::string("AFFIELD header is not valid JSON: ") + e.what());
  }
  const auto shape = f.header.at("shape").get<std::array<std::size_t, 3>>();
  const std::size_t n = element_count(shape);
  for (const auto& block : f.header.at("blocks")) {
    const auto offset = block.at("offset").get<std::size_t>();
    const auto length = block.at("length").get<std::size_t>();
    if (offset % kAlign != 0 || offset > bytes.size() || length > bytes.size() - offset) {
      throw FieldError("AFFIELD block '" + block.at("name").get<std::string>() + "' is out of bounds");
    }
    const std::string name = block.at("name");
    if (name == "values") {
      if (block.at("dtype") != "float32" || length != n * sizeof(float)) throw FieldError("bad values block");
      f.values.resize(n);
      std::memcpy(f.values.data(), bytes.data() + offset, length);
    } else if (name == "valid") {
      if (block.at("dtype") != "uint8" || length != n) throw FieldError("bad valid block");
      f.valid.assign(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
                     bytes.begin() + static_cast<std::ptrdiff_t>(offset + length));
    }
  }
  if (f.values.size() != n || f.valid.size() != n) throw FieldError("AFFIELD file is missing a data block");
  return f;
}

}  // namespace aeroflow
