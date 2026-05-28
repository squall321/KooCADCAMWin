// @lat: [[engine/skills#nitride]]

#include "nitride.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <string>

namespace koocadcam::skill::nitride {

using nlohmann::json;

namespace {

bool isKnownNitrideType(const std::string& t)
{
    return t == "gas" || t == "plasma" || t == "salt_bath";
}

// Industry-standard nitridable substrates: nitriding steels (Nitralloy,
// 4140, 4340, 8620), tool steels (H13, D2), some stainless.  Aluminum and
// brass do NOT respond — emit info, do not block.
bool isNitridableFamily(const std::string& material)
{
    std::string lc = material;
    std::transform(lc.begin(), lc.end(), lc.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lc.find("steel_4140")    != std::string::npos
        || lc.find("steel_4340")    != std::string::npos
        || lc.find("steel_8620")    != std::string::npos
        || lc.find("h13")           != std::string::npos
        || lc.find("d2")            != std::string::npos
        || lc.find("nitralloy")     != std::string::npos
        || lc.find("nitride_grade") != std::string::npos
        || lc.find("alloy_steel")   != std::string::npos
        || lc.find("tool_steel")    != std::string::npos
        || lc.find("stainless")     != std::string::npos;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.temperature_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": temperature_c must be > 0 (got " +
              std::to_string(in.temperature_c) + ")");
    }
    if (in.time_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": time_h must be > 0 (got " +
              std::to_string(in.time_h) + ")");
    }

    if (in.case_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": case_depth_mm must be > 0");
    } else if (in.case_depth_mm < 0.02) {
        r.add("DFM-NITRIDE-CASE", "error",
              std::string(kSkillId) + ": case_depth_mm " +
              std::to_string(in.case_depth_mm) +
              " < 0.02 mm — below practical nitride minimum");
    } else if (in.case_depth_mm > 1.0) {
        r.add("DFM-NITRIDE-CASE", "error",
              std::string(kSkillId) + ": case_depth_mm " +
              std::to_string(in.case_depth_mm) +
              " > 1.0 mm — exceeds typical nitride depth ceiling");
    }

    if (!isKnownNitrideType(in.type)) {
        r.add("DFM-NITRIDE-TYPE", "info",
              std::string(kSkillId) + ": type '" + in.type +
              "' unrecognised — expected gas | plasma | salt_bath");
    }

    if (in.temperature_c > 0.0 &&
        (in.temperature_c < 480.0 || in.temperature_c > 600.0)) {
        r.add("DFM-NITRIDE-TEMP", "info",
              std::string(kSkillId) + ": temperature_c " +
              std::to_string(in.temperature_c) +
              " °C outside typical nitride window [480, 600] °C — "
              "above 600 °C risks tempering core hardness");
    }

    if (!isNitridableFamily(wp.material())) {
        r.add("DFM-NITRIDE-MATERIAL", "info",
              std::string(kSkillId) + ": substrate material '" + wp.material() +
              "' not in standard nitridable family — case may not form");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "nitride DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "material",      in.material },
        { "temperature_c", in.temperature_c },
        { "time_h",        in.time_h },
        { "case_depth_mm", in.case_depth_mm },
        { "type",          in.type },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "material",         in.material },
        { "temperature_c",    in.temperature_c },
        { "time_h",           in.time_h },
        { "case_depth_mm",    in.case_depth_mm },
        { "type",             in.type },
        { "geometry_changed", false },
        { "requires_temper",  false },   // no quench → no temper needed
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "nitride_furnace";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.type == "salt_bath" ? "molten_cyanate_salt"
                              : in.type == "plasma"    ? "plasma_chamber"
                                                       : "ammonia_atmosphere";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;          // diffusion-only — no removal
    // Cycle time ≈ time_h × 3600 + 30-min ramp + 30-min cool buffer.
    tooling.est_cycle_time_s  = in.time_h * 3600.0 + 3600.0;
    tooling.extra = {
        { "process",          std::string("nitride_") + in.type },
        { "temperature_c",    in.temperature_c },
        { "time_h",           in.time_h },
        { "case_depth_mm",    in.case_depth_mm },
        { "type",             in.type },
        { "process_category", "case_hardening" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
        { "effect",           "nitrogen diffusion → hard surface case" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::nitride applied: material={} T={}°C t={}h case={}mm type={}",
                  in.material, in.temperature_c, in.time_h,
                  in.case_depth_mm, in.type);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::nitride
