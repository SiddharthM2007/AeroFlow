#pragma once

namespace aeroflow {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  friend constexpr bool operator==(const Vec3&, const Vec3&) = default;
};

}  // namespace aeroflow
