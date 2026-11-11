#pragma once

#include <cstdint>
#include <span>
#include <string>

namespace aeroflow {

// Standard base64 (RFC 4648) with '=' padding.
std::string base64_encode(std::span<const std::uint8_t> bytes);

}  // namespace aeroflow
