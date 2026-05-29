// @lat: [[engine/skills#ballast_install]]

#include "ballast_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::ballast_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.ballast_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": ballast_type must be non-empty");
    }

    if (in.total_mass_t <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": total_mass_t must be > 0 (got " +
              std::to_string(in.total_mass_t) + ")");
    }

    if (in.locations_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": locations_count must be > 0 (got " +
              std::to_string(in.locations_count) + ")");
    }

    if (in.total_mass_t > 500.0) {
        r.add("DFM-BALLAST-MASS", "info",
              std::string(kSkillId) + ": total_mass_t " +
              std::to_string(in.total_mass_t) +
              " > 500 — heavy-lift planning + dock floor load review advisory");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ballast_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double massPerLocation =
        in.locations_count > 0
            ? in.total_mass_t / static_cast<double>(in.locations_count)
            : 0.0;

    json params = {
        { "ballast_type",    in.ballast_type },
        { "total_mass_t",    in.total_mass_t },
        { "locations_count", in.locations_count },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "ballast_type",          in.ballast_type },
        { "total_mass_t",          in.total_mass_t },
        { "locations_count",       in.locations_count },
        { "mass_per_location_t",   massPerLocation },
        { "geometry_changed",      false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "ballast_handling_rig";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.ballast_type;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Ballast is ADDED mass — record as negative removal, mass-balance
    // convention (mirrors concrete_pour).  Pig iron ≈ 7.0 t/m3.
    tooling.stock_removed_mm3 = -(in.total_mass_t / 7.0) * 1.0e9;
    // Allow ~ 2 h per location to crane + secure ballast.
    tooling.est_cycle_time_s  =
        static_cast<double>(in.locations_count) * 2.0 * 3600.0;
    tooling.extra = {
        { "process",          "fixed_ballast_install" },
        { "ballast_type",     in.ballast_type },
        { "total_mass_t",     in.total_mass_t },
        { "locations_count",  in.locations_count },
        { "code_reference",   "SOLAS_II-1_stability" },
        { "process_category", "shipbuilding_outfit" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::ballast_install applied: type={} mass={}t locations={}",
        in.ballast_type, in.total_mass_t, in.locations_count);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != std::string(kSkillId)) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::ballast_install
