#include <gtest/gtest.h>

#include <string>

#include "aeroflow/config.hpp"

using aeroflow::ConfigError;
using aeroflow::parse_config_text;
using nlohmann::json;

namespace {

json valid_config() {
  return json::parse(R"({
    "domain": {"lx": 4.0, "ly": 2.0, "lz": 2.0},
    "resolution": {"nx": 40, "ny": 20, "nz": 20},
    "test_box": {"min": [1.0, 0.5, 0.5], "max": [2.0, 1.5, 1.5]}
  })");
}

// Expects parsing to fail with a message containing `fragment`.
void expect_error(const json& config, const std::string& fragment) {
  try {
    (void)aeroflow::parse_config(config);
    FAIL() << "expected ConfigError containing '" << fragment << "'";
  } catch (const ConfigError& e) {
    EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos)
        << "message was: " << e.what();
  }
}

}  // namespace

TEST(Config, ParsesValidConfig) {
  const auto c = aeroflow::parse_config(valid_config());
  EXPECT_EQ(c.nx, 40u);
  EXPECT_EQ(c.ny, 20u);
  EXPECT_EQ(c.nz, 20u);
  EXPECT_DOUBLE_EQ(c.lx, 4.0);
  ASSERT_TRUE(c.test_box.has_value());
  EXPECT_DOUBLE_EQ(c.test_box->min.x, 1.0);
  EXPECT_DOUBLE_EQ(c.test_box->max.z, 1.5);
}

TEST(Config, TestBoxIsOptional) {
  auto j = valid_config();
  j.erase("test_box");
  EXPECT_FALSE(aeroflow::parse_config(j).test_box.has_value());
  j["test_box"] = nullptr;
  EXPECT_FALSE(aeroflow::parse_config(j).test_box.has_value());
}

TEST(Config, RejectsMalformedJson) {
  EXPECT_THROW(parse_config_text("{not json"), ConfigError);
  EXPECT_THROW(parse_config_text("[1, 2, 3]"), ConfigError);
}

TEST(Config, RejectsMissingSections) {
  auto j = valid_config();
  j.erase("domain");
  expect_error(j, "domain: is required");
  j = valid_config();
  j["resolution"].erase("ny");
  expect_error(j, "resolution.ny: is required");
}

TEST(Config, RejectsUnknownFields) {
  auto j = valid_config();
  j["solver"] = "fast";
  expect_error(j, "solver: unknown field");
  j = valid_config();
  j["domain"]["lw"] = 1.0;
  expect_error(j, "domain.lw: unknown field");
}

TEST(Config, RejectsBadLengths) {
  for (const json bad : {json(0.0), json(-1.0), json("4"), json(nullptr)}) {
    auto j = valid_config();
    j["domain"]["lx"] = bad;
    EXPECT_THROW((void)aeroflow::parse_config(j), ConfigError) << bad.dump();
  }
}

TEST(Config, RejectsBadCounts) {
  for (const json bad : {json(0), json(-3), json(2.5), json(64.0), json("8")}) {
    auto j = valid_config();
    j["resolution"]["nx"] = bad;
    EXPECT_THROW((void)aeroflow::parse_config(j), ConfigError) << bad.dump();
  }
}

TEST(Config, RejectsTooManyCells) {
  auto j = valid_config();
  j["resolution"] = {{"nx", 1024}, {"ny", 1024}, {"nz", 1024}};
  expect_error(j, "must not exceed");
}

TEST(Config, RejectsInvertedBox) {
  auto j = valid_config();
  j["test_box"]["min"][1] = 1.5;
  j["test_box"]["max"][1] = 0.5;
  expect_error(j, "min.y must be less than max.y");
}

TEST(Config, RejectsDegenerateBox) {
  auto j = valid_config();
  j["test_box"]["max"][0] = 1.0;  // equal to min.x
  expect_error(j, "min.x must be less than max.x");
}

TEST(Config, RejectsBoxOutsideDomain) {
  auto j = valid_config();
  j["test_box"]["max"][2] = 2.5;  // lz = 2
  expect_error(j, "extent along z must lie within the domain");
  j = valid_config();
  j["test_box"]["min"][0] = -0.1;
  expect_error(j, "extent along x must lie within the domain");
}

TEST(Config, RejectsMalformedBoxPoints) {
  auto j = valid_config();
  j["test_box"]["min"] = {1.0, 2.0};
  expect_error(j, "test_box.min: must be an array of 3 numbers");
}
