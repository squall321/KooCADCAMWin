#pragma once
// @lat: [[engine/feature-watch#JSON Schema]]

#include <nlohmann/json_fwd.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace koocadcam::io {

class JsonSpec
{
public:
    // Read a JSON file from disk and parse. Returns nullopt on parse/IO failure.
    // On failure, populates `err` with a human-readable message.
    static std::optional<nlohmann::json> read(
        const std::filesystem::path& path, std::string& err) noexcept;

    // Write a JSON document to disk (4-space indent, sorted keys for determinism).
    // Returns true on success.
    static bool write(const nlohmann::json& spec,
                      const std::filesystem::path& path,
                      std::string& err) noexcept;

    // Light validation of a Watch spec: required field presence + types + ranges
    // matching feature-watch.md JSON Schema (M1.1 subset: schema_version,
    // product_name, form_factor, base, corner_radius, bezel).
    // Full JSON Schema validator is M3 work.
    // Returns true if valid; otherwise populates `errors` with one entry per issue.
    static bool validateWatchSpec(const nlohmann::json& spec,
                                  std::vector<std::string>& errors);
};

}  // namespace koocadcam::io
