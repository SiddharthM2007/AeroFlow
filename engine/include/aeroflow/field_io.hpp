#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "aeroflow/grid3d.hpp"
#include "aeroflow/mac_grid.hpp"

namespace aeroflow {

// AFFIELD v1 — canonical binary field file shared by the engine, backend and frontend.
//
//   bytes 0..7     magic "AFFIELD\0"
//   bytes 8..11    uint32 little-endian format version (1)
//   bytes 12..15   uint32 little-endian header length H
//   bytes 16..16+H UTF-8 JSON header, space-padded so 16 + H is a multiple of 8
//   then           data blocks at the absolute byte offsets listed in header["blocks"]
//
// Blocks: "values" (float32 LE, one per sample) and "valid" (uint8, 1 = valid sample), both in
// the field's own x-fastest order idx = i + n0*(j + n1*k) over header["shape"] = [n0, n1, n2].
// Values are never transposed or resampled. Statistics in the header are computed in float64
// before the values are narrowed to float32.
inline constexpr char kFieldMagic[8] = {'A', 'F', 'F', 'I', 'E', 'L', 'D', '\0'};
inline constexpr std::uint32_t kFieldFormatVersion = 1;

class FieldError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

enum class FieldLocation { Cell, FaceX, FaceY, FaceZ };

const char* to_string(FieldLocation location) noexcept;

// Array extents of a field sampled at `location` on an Nx x Ny x Nz cell grid.
std::array<std::size_t, 3> field_shape(FieldLocation location, std::size_t nx, std::size_t ny, std::size_t nz);

struct FieldStats {
  std::size_t valid_count = 0;
  double min = 0.0;
  double max = 0.0;
  double max_abs = 0.0;
  double mean_abs = 0.0;
  double rms = 0.0;
};

// Statistics over valid samples, in float64. valid_count == 0 means "no statistics".
FieldStats compute_field_stats(std::span<const double> values, std::span<const std::uint8_t> valid);

struct ScalarField {
  std::string name;       // unique id, e.g. "divergence_diagnostic"
  std::string quantity;   // physical quantity, e.g. "divergence"
  std::string long_name;  // human-readable label
  std::string units;      // e.g. "1/s"
  // Whether the quantity can be negative (divergence, pressure) or is nonnegative (magnitudes).
  // Lets viewers pick a zero-centered diverging or a sequential color map without guessing.
  bool is_signed = true;
  FieldLocation location = FieldLocation::Cell;
  std::array<std::size_t, 3> shape{};
  std::vector<double> values;        // float64 source, x-fastest over `shape`
  std::vector<std::uint8_t> valid;   // 1 = valid sample
  nlohmann::json frame;              // {"index": 0, "time_s": null, "label": "..."}
  nlohmann::json provenance;         // {"kind": "diagnostic" | "solution", "solved": bool, ...}
  nlohmann::json mask;               // how validity was derived
};

// Builds the JSON header (without "blocks") for `field` on the given grid.
nlohmann::json field_header(const ScalarField& field, const MacGrid& grid, const FieldStats& stats);

// Writes an AFFIELD v1 file; returns the complete header (including block offsets).
// Throws FieldError on I/O errors or inconsistent sizes.
nlohmann::json write_field_file(const std::string& path, const ScalarField& field, const MacGrid& grid);

struct FieldFile {
  nlohmann::json header;
  std::vector<float> values;
  std::vector<std::uint8_t> valid;
};

// Reads and validates an AFFIELD file (magic, version, header, block bounds and sizes).
FieldFile read_field_file(const std::string& path);

}  // namespace aeroflow
