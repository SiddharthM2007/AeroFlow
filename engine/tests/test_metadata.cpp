#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "aeroflow/base64.hpp"
#include "aeroflow/metadata.hpp"

using nlohmann::json;

TEST(Base64, MatchesRfc4648Vectors) {
  auto enc = [](const std::string& s) {
    const std::vector<std::uint8_t> bytes(s.begin(), s.end());
    return aeroflow::base64_encode(bytes);
  };
  EXPECT_EQ(enc(""), "");
  EXPECT_EQ(enc("f"), "Zg==");
  EXPECT_EQ(enc("fo"), "Zm8=");
  EXPECT_EQ(enc("foo"), "Zm9v");
  EXPECT_EQ(enc("foob"), "Zm9vYg==");
  EXPECT_EQ(enc("fooba"), "Zm9vYmE=");
  EXPECT_EQ(enc("foobar"), "Zm9vYmFy");
}

TEST(Metadata, ReportsGridAndClassification) {
  const auto config = aeroflow::parse_config(json::parse(R"({
    "domain": {"lx": 4.0, "ly": 4.0, "lz": 4.0},
    "resolution": {"nx": 4, "ny": 4, "nz": 4},
    "test_box": {"min": [1.0, 1.0, 1.0], "max": [3.0, 3.0, 3.0]}
  })"));
  const auto grid = aeroflow::build_grid(config);
  const auto meta = aeroflow::grid_metadata(grid, config, true);

  EXPECT_EQ(meta["grid"]["cell_count"], 64);
  EXPECT_DOUBLE_EQ(meta["grid"]["spacing"]["dx"].get<double>(), 1.0);
  EXPECT_EQ(meta["classification"]["solid_count"], 8);
  EXPECT_EQ(meta["classification"]["fluid_count"], 56);
  EXPECT_EQ(meta["classification"]["encoding"], "base64-uint8");
  // 64 bytes -> 88 base64 chars (with padding).
  EXPECT_EQ(meta["classification"]["data"].get<std::string>().size(), 88u);
  EXPECT_TRUE(meta["warnings"].empty());
}

TEST(Metadata, OmitsCellsAndWarnsOnEmptyBox) {
  const auto config = aeroflow::parse_config(json::parse(R"({
    "domain": {"lx": 4.0, "ly": 4.0, "lz": 4.0},
    "resolution": {"nx": 4, "ny": 4, "nz": 4},
    "test_box": {"min": [0.6, 0.6, 0.6], "max": [1.4, 1.4, 1.4]}
  })"));
  const auto meta = aeroflow::grid_metadata(aeroflow::build_grid(config), config, false);
  EXPECT_FALSE(meta["classification"].contains("data"));
  EXPECT_EQ(meta["classification"]["solid_count"], 0);
  EXPECT_EQ(meta["warnings"].size(), 1u);
}
