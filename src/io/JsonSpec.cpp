// @lat: [[engine/feature-watch#JSON Schema]]

#include "io/JsonSpec.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <string>

namespace koocadcam::io {

std::optional<nlohmann::json> JsonSpec::read(
    const std::filesystem::path& path, std::string& err) noexcept
{
    std::ifstream ifs(path);
    if (!ifs) {
        err = "cannot open " + path.string();
        return std::nullopt;
    }
    try {
        return nlohmann::json::parse(ifs);
    } catch (const nlohmann::json::parse_error& e) {
        err = "JSON parse error in " + path.string() + ": " + e.what();
        return std::nullopt;
    } catch (const std::exception& e) {
        err = "unexpected error reading " + path.string() + ": " + e.what();
        return std::nullopt;
    }
}

bool JsonSpec::write(const nlohmann::json& spec,
                     const std::filesystem::path& path,
                     std::string& err) noexcept
{
    try {
        std::ofstream ofs(path);
        if (!ofs) {
            err = "cannot open for writing: " + path.string();
            return false;
        }
        ofs << spec.dump(4);
        if (!ofs) {
            err = "write error on: " + path.string();
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        err = "unexpected error writing " + path.string() + ": " + e.what();
        return false;
    }
}

// ---------------------------------------------------------------------------
// validateWatchSpec
// ---------------------------------------------------------------------------
namespace {

// Helper: check that a key exists and is numeric; if so return its double value.
// Pushes error and returns false otherwise.
bool requireNumber(const nlohmann::json& obj,
                   const std::string& key,
                   const std::string& pointer,
                   const std::string& reason,
                   double& out,
                   std::vector<std::string>& errors)
{
    if (!obj.contains(key)) {
        errors.push_back(pointer + "/" + key + ": " + reason);
        return false;
    }
    if (!obj[key].is_number()) {
        errors.push_back(pointer + "/" + key + ": must be a number");
        return false;
    }
    out = obj[key].get<double>();
    return true;
}

void checkRange(double value,
                double lo,
                double hi,
                const std::string& pointer,
                std::vector<std::string>& errors)
{
    if (value < lo || value > hi) {
        errors.push_back(pointer + ": must be in [" +
                         std::to_string(lo) + ", " + std::to_string(hi) + "]" +
                         " (got " + std::to_string(value) + ")");
    }
}

}  // namespace

bool JsonSpec::validateWatchSpec(const nlohmann::json& spec,
                                 std::vector<std::string>& errors)
{
    // ── Required top-level fields ────────────────────────────────
    const std::vector<std::string> required = {
        "schema_version", "product_name", "form_factor",
        "base", "corner_radius", "bezel"
    };
    for (const auto& field : required) {
        if (!spec.contains(field)) {
            errors.push_back("/" + field + ": required field missing");
        }
    }

    // If any top-level required fields are missing we cannot continue safely.
    if (!errors.empty()) {
        return false;
    }

    // ── schema_version ───────────────────────────────────────────
    if (!spec["schema_version"].is_string() ||
        spec["schema_version"].get<std::string>() != "1.0.0") {
        errors.push_back("/schema_version: must be \"1.0.0\"");
    }

    // ── form_factor ──────────────────────────────────────────────
    if (!spec["form_factor"].is_string()) {
        errors.push_back("/form_factor: must be a string");
        return false;
    }
    const std::string formFactor = spec["form_factor"].get<std::string>();
    if (formFactor != "round" && formFactor != "square") {
        errors.push_back("/form_factor: must be \"round\" or \"square\"");
    }

    // ── base ─────────────────────────────────────────────────────
    const nlohmann::json& base = spec["base"];
    if (!base.is_object()) {
        errors.push_back("/base: must be an object");
    } else {
        if (formFactor == "round") {
            double diameterMm = 0.0;
            if (requireNumber(base, "diameter_mm", "/base",
                              "required for form_factor=round",
                              diameterMm, errors)) {
                checkRange(diameterMm, 30.0, 55.0, "/base/diameter_mm", errors);
            }
        } else if (formFactor == "square") {
            double widthMm = 0.0;
            if (requireNumber(base, "width_mm", "/base",
                              "required for form_factor=square",
                              widthMm, errors)) {
                checkRange(widthMm, 30.0, 60.0, "/base/width_mm", errors);
            }
            double heightMm = 0.0;
            if (requireNumber(base, "height_mm", "/base",
                              "required for form_factor=square",
                              heightMm, errors)) {
                checkRange(heightMm, 30.0, 60.0, "/base/height_mm", errors);
            }
        }

        double thicknessMm = 0.0;
        const bool hasThickness = requireNumber(base, "thickness_mm", "/base",
                                                "required", thicknessMm, errors);
        if (hasThickness) {
            checkRange(thicknessMm, 5.0, 20.0, "/base/thickness_mm", errors);
        }

        // ── bezel cross-rule (need thickness available) ──────────
        if (hasThickness && spec.contains("bezel") && spec["bezel"].is_object()) {
            const nlohmann::json& bezel = spec["bezel"];
            if (bezel.contains("depth_mm") && bezel["depth_mm"].is_number()) {
                const double bezelDepth = bezel["depth_mm"].get<double>();
                if (bezelDepth >= thicknessMm * 0.5) {
                    errors.push_back(
                        "/bezel/depth_mm: bezel depth too large relative to base thickness");
                }
            }
        }
    }

    // ── corner_radius ────────────────────────────────────────────
    const nlohmann::json& cr = spec["corner_radius"];
    if (!cr.is_object()) {
        errors.push_back("/corner_radius: must be an object");
    } else {
        double rTopMm = 0.0;
        if (requireNumber(cr, "r_top_mm", "/corner_radius",
                          "required", rTopMm, errors)) {
            checkRange(rTopMm, 0.2, 5.0, "/corner_radius/r_top_mm", errors);
        }
        double rSideMm = 0.0;
        if (requireNumber(cr, "r_side_mm", "/corner_radius",
                          "required", rSideMm, errors)) {
            checkRange(rSideMm, 0.2, 3.0, "/corner_radius/r_side_mm", errors);
        }
    }

    // ── bezel ────────────────────────────────────────────────────
    const nlohmann::json& bezel = spec["bezel"];
    if (!bezel.is_object()) {
        errors.push_back("/bezel: must be an object");
    } else {
        double widthMm = 0.0;
        if (requireNumber(bezel, "width_mm", "/bezel",
                          "required", widthMm, errors)) {
            checkRange(widthMm, 1.0, 8.0, "/bezel/width_mm", errors);
        }
        double depthMm = 0.0;
        if (requireNumber(bezel, "depth_mm", "/bezel",
                          "required", depthMm, errors)) {
            checkRange(depthMm, 0.3, 3.0, "/bezel/depth_mm", errors);
        }
        // Optional taper_deg
        if (bezel.contains("taper_deg") && !bezel["taper_deg"].is_null()) {
            if (!bezel["taper_deg"].is_number()) {
                errors.push_back("/bezel/taper_deg: must be a number");
            } else {
                const double taperDeg = bezel["taper_deg"].get<double>();
                checkRange(taperDeg, 0.0, 10.0, "/bezel/taper_deg", errors);
            }
        }
    }

    return errors.empty();
}

}  // namespace koocadcam::io
