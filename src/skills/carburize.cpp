// @lat: [[engine/skills#carburize]]

#include "carburize.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <string>

namespace koocadcam::skill::carburize {

using nlohmann::json;

namespace {

// Materials traditionally specified for case carburize.  Substring matched
// against Workpiece::material(); info-only — never blocks.
bool isCarburizableFamily(const std::string& material)
{
    std::string lc = material;
    std::transform(lc.begin(), lc.end(), lc.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    // Low-carbon steels (8620, 4320, 9310, 1018, 1020, …) and case-grade
    // steels are the standard carburize substrates.  High-carbon steels do
    // NOT respond — they should be through-hardened instead.
    return lc.find("steel_1018")     != std::string::npos
        || lc.find("steel_1020")     != std::string::npos
        || lc.find("steel_8620")     != std::string::npos
        || lc.find("steel_4320")     != std::string::npos
        || lc.find("steel_9310")     != std::string::npos
        || lc.find("low_carbon")     != std::string::npos
        || lc.find("case_steel")     != std::string::npos
        || lc.find("carburize_grade") != std::string::npos;
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
    } else if (in.case_depth_mm < 0.05) {
        r.add("DFM-CARBURIZE-CASE", "error",
              std::string(kSkillId) + ": case_depth_mm " +
              std::to_string(in.case_depth_mm) +
              " < 0.05 mm — below practical industrial minimum");
    } else if (in.case_depth_mm > 3.0) {
        r.add("DFM-CARBURIZE-CASE", "error",
              std::string(kSkillId) + ": case_depth_mm " +
              std::to_string(in.case_depth_mm) +
              " > 3.0 mm — exceeds typical commercial carburize depth ceiling");
    }

    if (in.target_hardness_hrc <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": target_hardness_hrc must be > 0");
    } else if (in.target_hardness_hrc < 20.0 || in.target_hardness_hrc > 70.0) {
        r.add("DFM-CARBURIZE-HARDNESS", "error",
              std::string(kSkillId) + ": target_hardness_hrc " +
              std::to_string(in.target_hardness_hrc) +
              " outside Rockwell-C valid window [20, 70]");
    }

    if (in.temperature_c > 0.0 &&
        (in.temperature_c < 800.0 || in.temperature_c > 1000.0)) {
        r.add("DFM-CARBURIZE-TEMP", "info",
              std::string(kSkillId) + ": temperature_c " +
              std::to_string(in.temperature_c) +
              " °C outside typical carburize window [800, 1000] °C");
    }

    if (!isCarburizableFamily(wp.material())) {
        r.add("DFM-CARBURIZE-MATERIAL", "info",
              std::string(kSkillId) + ": substrate material '" + wp.material() +
              "' not in low-carbon carburize family — case may not develop "
              "(consider through-hardening with 'harden' instead)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "carburize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "material",            in.material },
        { "temperature_c",       in.temperature_c },
        { "time_h",              in.time_h },
        { "case_depth_mm",       in.case_depth_mm },
        { "target_hardness_hrc", in.target_hardness_hrc },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "material",            in.material },
        { "temperature_c",       in.temperature_c },
        { "time_h",              in.time_h },
        { "case_depth_mm",       in.case_depth_mm },
        { "target_hardness_hrc", in.target_hardness_hrc },
        { "geometry_changed",    false },
        { "requires_temper",     true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "carburize_furnace";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "refractory_lining";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;          // diffusion-only — no removal
    // Cycle time ≈ time_h × 3600 + 1 h ramp + 1 h cool buffer.
    tooling.est_cycle_time_s  = in.time_h * 3600.0 + 2.0 * 3600.0;
    tooling.extra = {
        { "process",              "gas_carburize" },
        { "temperature_c",        in.temperature_c },
        { "time_h",               in.time_h },
        { "case_depth_mm",        in.case_depth_mm },
        { "target_hardness_hrc",  in.target_hardness_hrc },
        { "process_category",     "case_hardening" },
        { "geometric_no_op",      true },
        { "followup_recommended", "quench_then_temper" },
        { "dfm_findings",         findings },
        { "effect",               "carbon diffusion → hard surface case" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::carburize applied: material={} T={}°C t={}h "
                  "case={}mm target_HRC={}",
                  in.material, in.temperature_c, in.time_h,
                  in.case_depth_mm, in.target_hardness_hrc);

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

}  // namespace koocadcam::skill::carburize
