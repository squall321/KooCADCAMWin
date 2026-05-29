// @lat: [[engine/skills#insulation_install]]

#include "insulation_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill::insulation_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thickness_mm <= 0.0 || in.thickness_mm > 500.0) {
        r.add("DFM-INSUL-THK", "error",
              std::string(kSkillId) + ": thickness_mm " +
              std::to_string(in.thickness_mm) +
              " outside (0, 500] mm — non-standard insulation layer");
    }

    if (in.area_m2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": area_m2 must be > 0");
    }

    if (in.r_value <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": r_value must be > 0");
    } else if (in.r_value < 1.0) {
        r.add("DFM-INSUL-PERF", "info",
              std::string(kSkillId) + ": r_value " +
              std::to_string(in.r_value) +
              " m^2K/W < 1.0 — likely sub-code thermal performance");
    }

    static const std::vector<std::string> known =
        { "mineral_wool", "rigid_foam", "pir", "fibreglass", "rockwool",
          "cellulose", "xps", "eps", "aerogel" };
    bool found = false;
    for (const auto& m : known) if (m == in.insulation_type) { found = true; break; }
    if (!found && !in.insulation_type.empty()) {
        r.add("DFM-INSUL-MATERIAL", "info",
              std::string(kSkillId) + ": insulation_type '" +
              in.insulation_type + "' not in standard set");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "insulation_type", in.insulation_type },
        { "thickness_mm",    in.thickness_mm },
        { "area_m2",         in.area_m2 },
        { "r_value",         in.r_value },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "insulation_type",  in.insulation_type },
        { "thickness_mm",     in.thickness_mm },
        { "area_m2",          in.area_m2 },
        { "r_value",          in.r_value },
        { "u_value",
          (in.r_value > 0.0) ? (1.0 / in.r_value) : 0.0 },
        { "thermal_class",
          (in.r_value >= 1.0)
              ? std::string("code_compliant")
              : std::string("sub_code") },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "insulation_install_crew";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.thickness_mm;
    tooling.tool_material     = in.insulation_type;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 3 min / m^2 for batt or board insulation.
    tooling.est_cycle_time_s  = in.area_m2 * 180.0;
    tooling.extra = {
        { "process",         "insulation_install" },
        { "insulation_type", in.insulation_type },
        { "thickness_mm",    in.thickness_mm },
        { "area_m2",         in.area_m2 },
        { "r_value",         in.r_value },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::insulation_install applied: type={} thk={} area={} R={}",
                  in.insulation_type, in.thickness_mm, in.area_m2, in.r_value);

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

}  // namespace koocadcam::skill::insulation_install
