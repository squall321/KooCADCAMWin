// @lat: [[process/test-strategy#schema]]
// Breakthrough plan B6.1 — product-spec JSON Schema conformance tests.
//
// data/schemas/{watch,phone}.schema.json are the published contracts for the
// specs consumed by WatchFrontModel / PhoneFrontModel.  This test pins the
// engine defaults (defaultSpec) and the shipped sample fixtures to those
// contracts so a builder/schema drift fails CI immediately.
//
// A minimal JSON-Schema validator (Draft 2020-12 subset) is implemented
// below ON PURPOSE: the project must not grow an external json-schema-
// validator dependency.  Supported keywords:
//   type (object/array/string/number/integer/boolean),
//   properties, required, minimum, maximum, enum, items.
// All other keywords ($schema, $id, title, description, ...) are ignored,
// exactly as a real validator treats annotations.

#include <gtest/gtest.h>

#include "engine/PhoneFrontModel.hpp"
#include "engine/WatchFrontModel.hpp"
#include "io/JsonSpec.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

// ── Mini JSON-Schema validator ──────────────────────────────────────────────

bool typeMatches(const std::string& type, const nlohmann::json& v)
{
    if (type == "object")  return v.is_object();
    if (type == "array")   return v.is_array();
    if (type == "string")  return v.is_string();
    if (type == "boolean") return v.is_boolean();
    if (type == "integer") return v.is_number_integer();  // signed + unsigned
    if (type == "number")  return v.is_number();           // integers count too
    return false;  // unknown type name in the schema — fail loudly
}

void validateNode(const nlohmann::json& schema,
                  const nlohmann::json& value,
                  const std::string& path,
                  std::vector<std::string>& errors)
{
    const std::string where = path.empty() ? std::string("<root>") : path;

    // enum — deep equality against each candidate.
    if (schema.contains("enum")) {
        const auto& candidates = schema["enum"];
        const bool found = std::any_of(candidates.begin(), candidates.end(),
            [&value](const nlohmann::json& c) { return c == value; });
        if (!found) {
            errors.push_back(where + ": value " + value.dump() +
                             " not in enum " + candidates.dump());
        }
    }

    // type — on mismatch, skip children (they would only add noise).
    if (schema.contains("type")) {
        const std::string type = schema["type"].get<std::string>();
        if (!typeMatches(type, value)) {
            errors.push_back(where + ": expected type '" + type + "', got " +
                             value.dump());
            return;
        }
    }

    // minimum / maximum — numeric instances only (both bounds inclusive).
    if (value.is_number()) {
        const double d = value.get<double>();
        if (schema.contains("minimum")) {
            const double lo = schema["minimum"].get<double>();
            if (d < lo) {
                errors.push_back(where + ": " + std::to_string(d) +
                                 " < minimum " + std::to_string(lo));
            }
        }
        if (schema.contains("maximum")) {
            const double hi = schema["maximum"].get<double>();
            if (d > hi) {
                errors.push_back(where + ": " + std::to_string(d) +
                                 " > maximum " + std::to_string(hi));
            }
        }
    }

    // required + properties — object instances only.
    if (value.is_object()) {
        if (schema.contains("required")) {
            for (const auto& req : schema["required"]) {
                const std::string key = req.get<std::string>();
                if (!value.contains(key)) {
                    errors.push_back(path + "/" + key +
                                     ": required property missing");
                }
            }
        }
        if (schema.contains("properties")) {
            const auto& props = schema["properties"];
            for (auto it = props.begin(); it != props.end(); ++it) {
                if (value.contains(it.key())) {
                    validateNode(it.value(), value.at(it.key()),
                                 path + "/" + it.key(), errors);
                }
            }
        }
    }

    // items — array instances only (single-schema form).
    if (value.is_array() && schema.contains("items")) {
        std::size_t idx = 0;
        for (const auto& element : value) {
            validateNode(schema["items"], element,
                         path + "/" + std::to_string(idx), errors);
            ++idx;
        }
    }
}

std::vector<std::string> validate(const nlohmann::json& schema,
                                  const nlohmann::json& document)
{
    std::vector<std::string> errors;
    validateNode(schema, document, "", errors);
    return errors;
}

std::string joinErrors(const std::vector<std::string>& errors)
{
    std::string out;
    for (const auto& e : errors) {
        out += "\n  - " + e;
    }
    return out;
}

// ── File resolution ─────────────────────────────────────────────────────────

// Schema files live in <repo>/data/schemas.  CTest injects KOO_SCHEMA_DIR;
// running the exe by hand falls back to a parent-directory walk (same
// pattern as resolveFixture in tests/watch/watch_features_test.cpp).
fs::path resolveSchema(const char* leaf)
{
    if (const char* env = std::getenv("KOO_SCHEMA_DIR")) {
        const fs::path candidate = fs::path(env) / leaf;
        if (fs::exists(candidate)) return candidate;
    }
    fs::path p = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        const fs::path candidate = p / "data" / "schemas" / leaf;
        if (fs::exists(candidate)) return candidate;
        p = p.parent_path();
    }
    return {};
}

fs::path resolveFixture(const char* leaf)
{
    fs::path p = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        const fs::path candidate = p / "tests" / "data" / leaf;
        if (fs::exists(candidate)) return candidate;
        p = p.parent_path();
    }
    if (const char* env = std::getenv("KOO_TESTS_DATA_DIR")) {
        return fs::path(env) / leaf;
    }
    return {};
}

nlohmann::json loadJson(const fs::path& path)
{
    if (path.empty()) return {};
    std::string err;
    const auto opt = koocadcam::io::JsonSpec::read(path, err);
    return opt.has_value() ? *opt : nlohmann::json{};
}

}  // namespace

// 1. The engine's watch default spec conforms to the published watch schema.
TEST(SpecSchema, WatchDefaultSpecConforms)
{
    const nlohmann::json schema = loadJson(resolveSchema("watch.schema.json"));
    ASSERT_FALSE(schema.empty()) << "watch.schema.json not found "
                                    "(KOO_SCHEMA_DIR / data/schemas)";

    const nlohmann::json spec =
        koocadcam::engine::WatchFrontModel::defaultSpec();
    const auto errors = validate(schema, spec);
    EXPECT_TRUE(errors.empty()) << "watch defaultSpec violates schema:"
                                << joinErrors(errors);
}

// 2. The engine's phone default spec conforms to the published phone schema.
TEST(SpecSchema, PhoneDefaultSpecConforms)
{
    const nlohmann::json schema = loadJson(resolveSchema("phone.schema.json"));
    ASSERT_FALSE(schema.empty()) << "phone.schema.json not found "
                                    "(KOO_SCHEMA_DIR / data/schemas)";

    const nlohmann::json spec =
        koocadcam::engine::PhoneFrontModel::defaultSpec();
    const auto errors = validate(schema, spec);
    EXPECT_TRUE(errors.empty()) << "phone defaultSpec violates schema:"
                                << joinErrors(errors);
}

// 3. A deliberate violation (negative diameter) is rejected with a precise
//    error path — proves the validator is not a vacuous pass.
TEST(SpecSchema, NegativeDiameterIsRejected)
{
    const nlohmann::json schema = loadJson(resolveSchema("watch.schema.json"));
    ASSERT_FALSE(schema.empty());

    nlohmann::json bad = koocadcam::engine::WatchFrontModel::defaultSpec();
    bad["crown_cavity"]["diameter_mm"] = -5.0;

    const auto errors = validate(schema, bad);
    ASSERT_FALSE(errors.empty())
        << "schema accepted crown_cavity.diameter_mm = -5";
    const bool pathReported = std::any_of(errors.begin(), errors.end(),
        [](const std::string& e) {
            return e.find("/crown_cavity/diameter_mm") != std::string::npos;
        });
    EXPECT_TRUE(pathReported)
        << "violation reported without its JSON path:" << joinErrors(errors);
}

// 4. The shipped sample fixture conforms (covers keys defaultSpec shares
//    plus fixture-only extras like noise_seed, which the schema ignores).
TEST(SpecSchema, SampleWatchFixtureConforms)
{
    const nlohmann::json schema = loadJson(resolveSchema("watch.schema.json"));
    ASSERT_FALSE(schema.empty());

    const nlohmann::json spec = loadJson(resolveFixture("sample_watch.json"));
    ASSERT_FALSE(spec.empty()) << "tests/data/sample_watch.json not found";

    const auto errors = validate(schema, spec);
    EXPECT_TRUE(errors.empty()) << "sample_watch.json violates schema:"
                                << joinErrors(errors);
}
