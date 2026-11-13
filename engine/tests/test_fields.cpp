#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>

#include "aeroflow/box.hpp"
#include "aeroflow/config.hpp"
#include "aeroflow/divergence.hpp"
#include "aeroflow/field_export.hpp"
#include "aeroflow/field_io.hpp"

using namespace aeroflow;
namespace fs = std::filesystem;

namespace {

std::string temp_file(const char* name) { return (fs::temp_directory_path() / (std::string("aeroflow_") + name)).string(); }

std::string read_bytes(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

void write_bytes(const std::string& path, const std::string& bytes) {
  std::ofstream(path, std::ios::binary) << bytes;
}

// Golden field shared with backend/tests/test_fields.py and frontend/src/lib/fields/*.test.ts:
// 3 x 2 x 2 cells on 1.5 x 1.0 x 0.5 m; value(i,j,k) = i + 10 j + 100 k + 0.25; cell (1,1,0) invalid.
ScalarField golden_field() {
  ScalarField f;
  f.name = "golden";
  f.quantity = "test";
  f.long_name = "Golden test field";
  f.units = "m/s";
  f.is_signed = false;
  f.location = FieldLocation::Cell;
  f.shape = {3, 2, 2};
  for (std::size_t k = 0; k < 2; ++k)
    for (std::size_t j = 0; j < 2; ++j)
      for (std::size_t i = 0; i < 3; ++i) {
        const bool valid = !(i == 1 && j == 1 && k == 0);
        f.values.push_back(valid ? static_cast<double>(i + 10 * j + 100 * k) + 0.25 : 0.0);
        f.valid.push_back(valid ? 1 : 0);
      }
  f.frame = {{"index", 0}, {"time_s", nullptr}, {"label", "golden"}};
  f.mask = {{"source", "test"}, {"valid_rule", "all but (1,1,0)"}};
  f.provenance = {{"kind", "test"}, {"solved", false}};
  return f;
}

const MacGrid kGoldenGrid(3, 2, 2, 1.5, 1.0, 0.5);

}  // namespace

// --- AFFIELD format ---------------------------------------------------------------------------

TEST(FieldFormat, RoundTripsWithAlignedBlocks) {
  const auto path = temp_file("roundtrip.affield");
  const auto header = write_field_file(path, golden_field(), kGoldenGrid);
  const auto bytes = read_bytes(path);
  ASSERT_GE(bytes.size(), 16u);
  EXPECT_EQ(bytes.substr(0, 8), std::string("AFFIELD\0", 8));
  std::uint32_t version = 0, header_len = 0;
  std::memcpy(&version, bytes.data() + 8, 4);
  std::memcpy(&header_len, bytes.data() + 12, 4);
  EXPECT_EQ(version, 1u);
  EXPECT_EQ((16 + header_len) % 8, 0u);
  for (const auto& b : header["blocks"]) EXPECT_EQ(b["offset"].get<std::size_t>() % 8, 0u);

  const auto f = read_field_file(path);
  EXPECT_EQ(f.header["name"], "golden");
  EXPECT_EQ(f.header["location"], "cell");
  EXPECT_EQ(f.header["shape"], nlohmann::json::array({3, 2, 2}));
  EXPECT_EQ(f.header["index_order"], "i + n0*(j + n1*k)");
  EXPECT_EQ(f.header["byte_order"], "little");
  EXPECT_EQ(f.header["signed"], false);
  EXPECT_EQ(f.header["grid"]["spacing"], nlohmann::json::array({0.5, 0.5, 0.25}));
  ASSERT_EQ(f.values.size(), 12u);
  for (std::size_t k = 0; k < 2; ++k)
    for (std::size_t j = 0; j < 2; ++j)
      for (std::size_t i = 0; i < 3; ++i) {
        const std::size_t idx = i + 3 * (j + 2 * k);  // x-fastest, never transposed
        if (f.valid[idx]) EXPECT_EQ(f.values[idx], static_cast<float>(i + 10 * j + 100 * k) + 0.25f);
      }
  EXPECT_EQ(f.valid[1 + 3 * 1], 0);
  EXPECT_EQ(bytes.size(), header["blocks"][1]["offset"].get<std::size_t>() + 12);
  fs::remove(path);
}

TEST(FieldFormat, MatchesTheSharedGoldenFile) {
  // Set AEROFLOW_UPDATE_GOLDEN=1 to regenerate after an intentional format change (bump version).
  const std::string golden = std::string(AEROFLOW_FIXTURES_DIR) + "/fields/golden_v1.affield";
  const auto path = temp_file("golden.affield");
  write_field_file(path, golden_field(), kGoldenGrid);
  if (std::getenv("AEROFLOW_UPDATE_GOLDEN")) fs::copy_file(path, golden, fs::copy_options::overwrite_existing);
  ASSERT_TRUE(fs::exists(golden)) << golden;
  EXPECT_EQ(read_bytes(path), read_bytes(golden));
  fs::remove(path);
}

TEST(FieldFormat, StatisticsAreComputedInDoublePrecision) {
  auto f = golden_field();
  f.values[0] = 1.0 / 3.0;  // not representable in float32
  const auto path = temp_file("stats.affield");
  const auto header = write_field_file(path, f, kGoldenGrid);
  EXPECT_EQ(header["stats"]["min"].get<double>(), 1.0 / 3.0);
  EXPECT_EQ(header["stats"]["valid_count"], 11);
  EXPECT_EQ(header["stats"]["max"].get<double>(), 112.25);
  EXPECT_NE(static_cast<double>(read_field_file(path).values[0]), 1.0 / 3.0);  // transport is float32
  fs::remove(path);
}

TEST(FieldFormat, NoValidSamplesMeansNullStatistics) {
  auto f = golden_field();
  std::fill(f.valid.begin(), f.valid.end(), 0);
  const auto path = temp_file("empty.affield");
  const auto header = write_field_file(path, f, kGoldenGrid);
  EXPECT_EQ(header["stats"]["valid_count"], 0);
  EXPECT_TRUE(header["stats"]["max_abs"].is_null());
  EXPECT_TRUE(header["stats"]["rms"].is_null());
  fs::remove(path);
}

TEST(FieldFormat, RejectsInconsistentFields) {
  auto f = golden_field();
  f.values.pop_back();
  EXPECT_THROW(write_field_file(temp_file("bad.affield"), f, kGoldenGrid), FieldError);
  f = golden_field();
  f.shape = {4, 2, 2};
  EXPECT_THROW(write_field_file(temp_file("bad.affield"), f, kGoldenGrid), FieldError);
  f = golden_field();
  f.location = FieldLocation::FaceX;  // needs (nx+1) x ny x nz
  EXPECT_THROW(write_field_file(temp_file("bad.affield"), f, kGoldenGrid), FieldError);
}

TEST(FieldFormat, RejectsCorruptFiles) {
  const auto path = temp_file("corrupt.affield");
  write_field_file(path, golden_field(), kGoldenGrid);
  const auto good = read_bytes(path);

  auto bad = good;
  bad[0] = 'X';
  write_bytes(path, bad);
  EXPECT_THROW(read_field_file(path), FieldError);

  bad = good;
  bad[8] = 2;  // version 2
  write_bytes(path, bad);
  EXPECT_THROW(read_field_file(path), FieldError);

  write_bytes(path, good.substr(0, 40));  // truncated header
  EXPECT_THROW(read_field_file(path), FieldError);

  write_bytes(path, good.substr(0, good.size() - 3));  // truncated data
  EXPECT_THROW(read_field_file(path), FieldError);
  fs::remove(path);
}

TEST(FieldFormat, StaggeredShapes) {
  EXPECT_EQ(field_shape(FieldLocation::Cell, 3, 4, 5), (std::array<std::size_t, 3>{3, 4, 5}));
  EXPECT_EQ(field_shape(FieldLocation::FaceX, 3, 4, 5), (std::array<std::size_t, 3>{4, 4, 5}));
  EXPECT_EQ(field_shape(FieldLocation::FaceY, 3, 4, 5), (std::array<std::size_t, 3>{3, 5, 5}));
  EXPECT_EQ(field_shape(FieldLocation::FaceZ, 3, 4, 5), (std::array<std::size_t, 3>{3, 4, 6}));
}

// --- divergence diagnostic ------------------------------------------------------------------

TEST(DivergenceDiagnostic, UnobstructedFlowIsExactlyDivergenceFree) {
  const Grid3D mask(10, 6, 7, 2.0, 0.9, 1.4);  // non-cubic
  const auto f = divergence_diagnostic(mask);
  EXPECT_EQ(f.shape, (std::array<std::size_t, 3>{10, 6, 7}));
  for (double v : f.values) EXPECT_EQ(v, 0.0);
  for (auto ok : f.valid) EXPECT_EQ(ok, 1);
  EXPECT_EQ(f.provenance["kind"], "diagnostic");
  EXPECT_EQ(f.provenance["solved"], false);
  EXPECT_EQ(f.units, "1/s");
}

TEST(DivergenceDiagnostic, DivergenceIsLocalizedAtTheObstacle) {
  Grid3D mask(12, 8, 8, 2.4, 1.6, 1.6);  // dx = 0.2
  mark_solid_box(mask, {{0.8, 0.6, 0.6}, {1.6, 1.0, 1.0}});  // i 4..7, j 3..4, k 3..4
  const auto f = divergence_diagnostic(mask);
  const double jump = kDiagnosticInflowSpeed / mask.dx();  // 25 1/s
  for (std::size_t k = 0; k < 8; ++k)
    for (std::size_t j = 0; j < 8; ++j)
      for (std::size_t i = 0; i < 12; ++i) {
        const std::size_t idx = mask.index(i, j, k);
        const bool in_shadow = j >= 3 && j <= 4 && k >= 3 && k <= 4;
        if (mask.cell(i, j, k) == CellType::Solid) {
          EXPECT_EQ(f.valid[idx], 0);
          EXPECT_EQ(f.values[idx], 0.0);
        } else if (in_shadow && i == 3) {
          EXPECT_DOUBLE_EQ(f.values[idx], -jump);  // flow into a blocked face: sink
        } else if (in_shadow && i == 8) {
          EXPECT_DOUBLE_EQ(f.values[idx], jump);  // flow out of a blocked face: source
        } else {
          EXPECT_EQ(f.values[idx], 0.0) << i << "," << j << "," << k;
        }
      }
}

TEST(DivergenceDiagnostic, MatchesTheMacOperatorExactly) {
  Grid3D mask(9, 5, 4, 1.8, 0.75, 0.6);
  mark_solid_box(mask, {{0.6, 0.3, 0.15}, {1.2, 0.6, 0.45}});
  MacGrid mac(mask);
  const auto faces = classify_faces(mask);
  mac.set_uniform_velocity({kDiagnosticInflowSpeed, 0, 0}, FaceSelection::ActiveOnly, &faces);
  const auto div = compute_divergence(mac, mask);
  const auto f = divergence_diagnostic(mask);
  ASSERT_EQ(f.values.size(), div.values.size());
  for (std::size_t n = 0; n < f.values.size(); ++n) EXPECT_EQ(f.values[n], div.values.data()[n]);
}

TEST(FieldExport, WritesRequestedFieldsWithSummaries) {
  const auto dir = fs::temp_directory_path() / "aeroflow_fields_export";
  fs::create_directories(dir);
  Grid3D mask(6, 4, 4, 1.2, 0.8, 0.8);
  mark_solid_box(mask, {{0.4, 0.2, 0.2}, {0.8, 0.6, 0.6}});
  const auto summaries = export_fields(mask, {kDivergenceDiagnostic}, dir.string()).fields;
  ASSERT_EQ(summaries.size(), 1u);
  EXPECT_EQ(summaries[0]["file"], "divergence_diagnostic.f0.affield");
  EXPECT_EQ(summaries[0]["frame"]["index"], 0);
  EXPECT_GT(summaries[0]["stats"]["max_abs"].get<double>(), 0.0);
  EXPECT_EQ(summaries[0]["stats"]["valid_count"], mask.count(CellType::Fluid));
  const auto f = read_field_file((dir / "divergence_diagnostic.f0.affield").string());
  EXPECT_EQ(f.values.size(), mask.cell_count());

  EXPECT_THROW(export_fields(mask, {"vorticity"}, dir.string()), std::invalid_argument);
  EXPECT_THROW(export_fields(mask, {kDivergenceDiagnostic}, (dir / "missing").string()), FieldError);
  EXPECT_TRUE(export_fields(mask, {}, (dir / "missing").string()).fields.empty());
  fs::remove_all(dir);
}

TEST(FieldExport, ConfigValidation) {
  auto base = nlohmann::json::parse(R"({"domain": {"lx": 1, "ly": 1, "lz": 1}, "resolution": {"nx": 2, "ny": 2, "nz": 2}})");
  auto with = [&](nlohmann::json fields) {
    auto j = base;
    j["fields"] = std::move(fields);
    return j;
  };
  const auto ok = parse_config(with({{"output_dir", "/tmp/x"}, {"requested", {"divergence_diagnostic"}}}));
  ASSERT_TRUE(ok.fields.has_value());
  EXPECT_EQ(ok.fields->requested, std::vector<std::string>{"divergence_diagnostic"});
  EXPECT_THROW(parse_config(with({{"output_dir", "/tmp/x"}, {"requested", {"vorticity"}}})), ConfigError);
  EXPECT_THROW(parse_config(with({{"output_dir", "/tmp/x"},
                                  {"requested", {"divergence_diagnostic", "divergence_diagnostic"}}})),
               ConfigError);
  EXPECT_THROW(parse_config(with({{"requested", {"divergence_diagnostic"}}})), ConfigError);
  EXPECT_THROW(parse_config(with({{"output_dir", "/tmp/x"}, {"requested", "divergence_diagnostic"}})), ConfigError);
}
