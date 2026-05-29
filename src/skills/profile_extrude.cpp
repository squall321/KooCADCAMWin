// @lat: [[engine/skills#profile_extrude]]

#include "profile_extrude.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::profile_extrude {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.profile_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": profile_id must be non-empty");
    }

    if (in.cross_section_mm2 <= 0.0) {
        r.add("DFM-PROFILE-AREA", "error",
              std::string(kSkillId) + ": cross_section_mm2 must be > 0 (got " +
              std::to_string(in.cross_section_mm2) + ")");
    } else if (in.cross_section_mm2 > 50000.0) {
        r.add("DFM-PROFILE-AREA", "error",
              std::string(kSkillId) + ": cross_section_mm2 " +
              std::to_string(in.cross_section_mm2) +
              " > 50000 — beyond standard profile-die envelope");
    }

    if (in.take_off_speed_m_min <= 0.0) {
        r.add("DFM-PROFILE-SPEED", "error",
              std::string(kSkillId) + ": take_off_speed_m_min must be > 0 (got " +
              std::to_string(in.take_off_speed_m_min) + ")");
    } else if (in.take_off_speed_m_min > 60.0) {
        r.add("DFM-PROFILE-SPEED", "error",
              std::string(kSkillId) + ": take_off_speed_m_min " +
              std::to_string(in.take_off_speed_m_min) +
              " > 60 — calibrator cannot cool thick profile fast enough");
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

    // Mass output = ρ × A_mm2 × v_mm/h    with ρ ≈ 1.2e-6 kg/mm³ (rigid PVC)
    const double v_mm_h = in.take_off_speed_m_min * 60000.0;
    const double mass_output_kg_h =
        1.2e-6 * in.cross_section_mm2 * v_mm_h;

    json params = {
        { "profile_id",            in.profile_id },
        { "cross_section_mm2",     in.cross_section_mm2 },
        { "take_off_speed_m_min",  in.take_off_speed_m_min },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "profile_id",             in.profile_id },
        { "cross_section_mm2",      in.cross_section_mm2 },
        { "take_off_speed_m_min",   in.take_off_speed_m_min },
        { "mass_output_kg_h",       mass_output_kg_h },
        { "geometry_changed",       false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "profile_die_calibrator";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "hardened_tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 3600.0;
    tooling.extra = {
        { "process",               "polymer_profile_extrusion" },
        { "profile_id",            in.profile_id },
        { "cross_section_mm2",     in.cross_section_mm2 },
        { "take_off_speed_m_min",  in.take_off_speed_m_min },
        { "mass_output_kg_h",      mass_output_kg_h },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::profile_extrude applied: id={} A={}mm² v={}m/min",
                  in.profile_id, in.cross_section_mm2, in.take_off_speed_m_min);

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

}  // namespace koocadcam::skill::profile_extrude
