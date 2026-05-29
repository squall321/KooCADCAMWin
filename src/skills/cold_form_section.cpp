// @lat: [[engine/skills#cold_form_section]]

#include "cold_form_section.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::cold_form_section {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.section_profile != "C" &&
        in.section_profile != "Z" &&
        in.section_profile != "sigma") {
        r.add("DFM-CFS-PROF", "error",
              std::string(kSkillId) + ": section_profile '" + in.section_profile +
              "' unsupported — expected one of {C, Z, sigma}");
    }

    if (in.thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": thickness_mm must be > 0 (got " +
              std::to_string(in.thickness_mm) + ")");
    } else {
        if (in.thickness_mm < 0.5) {
            r.add("DFM-CFS-THK", "error",
                  std::string(kSkillId) + ": thickness_mm " +
                  std::to_string(in.thickness_mm) +
                  " < 0.5 mm — too thin for cold-form structural section");
        }
        if (in.thickness_mm > 4.0) {
            r.add("DFM-CFS-THK", "error",
                  std::string(kSkillId) + ": thickness_mm " +
                  std::to_string(in.thickness_mm) +
                  " > 4.0 mm — exceeds cold-form coil gauge, use hot-rolled");
        }
    }

    if (in.length_m <= 0.0) {
        r.add("DFM-CFS-LEN", "error",
              std::string(kSkillId) + ": length_m must be > 0 (got " +
              std::to_string(in.length_m) + ")");
    } else if (in.length_m > 18.0) {
        r.add("DFM-CFS-LEN", "error",
              std::string(kSkillId) + ": length_m " +
              std::to_string(in.length_m) +
              " > 18 m — exceeds typical roll-form line transport length");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cold_form_section DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "section_profile", in.section_profile },
        { "thickness_mm",    in.thickness_mm },
        { "length_m",        in.length_m },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "section_profile",  in.section_profile },
        { "thickness_mm",     in.thickness_mm },
        { "length_m",         in.length_m },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "roll_former";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.length_m * 1000.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Roll-form line ~ 30 m/min throughput.
    tooling.est_cycle_time_s  = in.length_m * 60.0 / 30.0;
    tooling.extra = {
        { "process",         "cold_roll_form" },
        { "section_profile", in.section_profile },
        { "thickness_mm",    in.thickness_mm },
        { "length_m",        in.length_m },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cold_form_section applied: profile={} t={} L={}",
                  in.section_profile, in.thickness_mm, in.length_m);

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

}  // namespace koocadcam::skill::cold_form_section
