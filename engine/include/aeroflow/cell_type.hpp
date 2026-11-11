#pragma once

#include <cstdint>
#include <string_view>

namespace aeroflow {

// Classification of a single grid cell. The underlying values are part of the
// CLI output contract (one byte per cell), so do not renumber them.
enum class CellType : std::uint8_t {
  Fluid = 0,
  Solid = 1,
};

constexpr std::string_view to_string(CellType type) noexcept {
  switch (type) {
    case CellType::Fluid: return "fluid";
    case CellType::Solid: return "solid";
  }
  return "unknown";
}

}  // namespace aeroflow
