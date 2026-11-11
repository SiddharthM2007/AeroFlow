#include "aeroflow/base64.hpp"

namespace aeroflow {

std::string base64_encode(std::span<const std::uint8_t> bytes) {
  static constexpr char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string out;
  out.reserve(((bytes.size() + 2) / 3) * 4);

  std::size_t n = 0;
  for (; n + 2 < bytes.size(); n += 3) {
    const std::uint32_t chunk = (std::uint32_t{bytes[n]} << 16) |
                                (std::uint32_t{bytes[n + 1]} << 8) | std::uint32_t{bytes[n + 2]};
    out.push_back(kAlphabet[(chunk >> 18) & 0x3F]);
    out.push_back(kAlphabet[(chunk >> 12) & 0x3F]);
    out.push_back(kAlphabet[(chunk >> 6) & 0x3F]);
    out.push_back(kAlphabet[chunk & 0x3F]);
  }

  const std::size_t remaining = bytes.size() - n;
  if (remaining == 1) {
    const std::uint32_t chunk = std::uint32_t{bytes[n]} << 16;
    out.push_back(kAlphabet[(chunk >> 18) & 0x3F]);
    out.push_back(kAlphabet[(chunk >> 12) & 0x3F]);
    out += "==";
  } else if (remaining == 2) {
    const std::uint32_t chunk = (std::uint32_t{bytes[n]} << 16) | (std::uint32_t{bytes[n + 1]} << 8);
    out.push_back(kAlphabet[(chunk >> 18) & 0x3F]);
    out.push_back(kAlphabet[(chunk >> 12) & 0x3F]);
    out.push_back(kAlphabet[(chunk >> 6) & 0x3F]);
    out.push_back('=');
  }
  return out;
}

}  // namespace aeroflow
