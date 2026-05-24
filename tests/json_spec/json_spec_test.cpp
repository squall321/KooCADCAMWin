// @lat: [[process/test-strategy#json_spec]]

#include <gtest/gtest.h>

#include "io/JsonSpec.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <vector>

namespace fs = std::filesystem;

namespace {
fs::path resolveFixture(const char* leaf)
{
    fs::path p = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        fs::path candidate = p / "tests" / "data" / leaf;
        if (fs::exists(candidate)) return candidate;
        p = p.parent_path();
    }
    if (const char* env = std::getenv("KOO_TESTS_DATA_DIR")) {
        return fs::path(env) / leaf;
    }
    return {};
}
}

TEST(JsonSpec, ReadWriteRoundTrip)
{
    using namespace koocadcam::io;
    const fs::path src = resolveFixture("sample_watch.json");
    ASSERT_FALSE(src.empty());
    std::string err;
    auto json1 = JsonSpec::read(src, err);
    ASSERT_TRUE(json1.has_value()) << err;

    const fs::path tmp = fs::temp_directory_path() / "koocadcam_json_spec_test.json";
    ASSERT_TRUE(JsonSpec::write(*json1, tmp, err)) << err;
    auto json2 = JsonSpec::read(tmp, err);
    ASSERT_TRUE(json2.has_value()) << err;
    EXPECT_EQ(*json1, *json2);
    std::error_code ec;
    fs::remove(tmp, ec);
}

TEST(JsonSpec, ValidateRejectsEmptyObject)
{
    using namespace koocadcam::io;
    nlohmann::json empty = nlohmann::json::object();
    std::vector<std::string> errors;
    EXPECT_FALSE(JsonSpec::validateWatchSpec(empty, errors));
    EXPECT_FALSE(errors.empty());
}

TEST(JsonSpec, ValidateAcceptsSampleWatch)
{
    using namespace koocadcam::io;
    auto path = resolveFixture("sample_watch.json");
    ASSERT_FALSE(path.empty());
    std::string err;
    auto j = JsonSpec::read(path, err);
    ASSERT_TRUE(j.has_value()) << err;
    std::vector<std::string> errors;
    EXPECT_TRUE(JsonSpec::validateWatchSpec(*j, errors))
        << "unexpected errors: " << (errors.empty() ? "" : errors.front());
}

TEST(JsonSpec, ValidateRejectsOutOfRange)
{
    using namespace koocadcam::io;
    nlohmann::json spec = {
        {"schema_version", "1.0.0"},
        {"product_name", "Bad"},
        {"form_factor", "round"},
        {"base", {{"diameter_mm", 200.0}, {"thickness_mm", 10.0}}},
        {"corner_radius", {{"r_top_mm", 1.0}, {"r_side_mm", 0.6}}},
        {"bezel", {{"width_mm", 3.0}, {"depth_mm", 1.0}}}
    };
    std::vector<std::string> errors;
    EXPECT_FALSE(JsonSpec::validateWatchSpec(spec, errors));
    EXPECT_FALSE(errors.empty());
}
