// @lat: [[engine/skills#closed_die_forge]]

#include "closed_die_forge.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::closed_die_forge {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shape_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": shape_id must not be empty");
    }
    if (in.flash_thickness_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": flash_thickness_mm must be >= 0 (got " +
              std::to_string(in.flash_thickness_mm) + ")");
    }
    if (in.flash_thickness_mm > 5.0) {
        r.add("DFM-CDF-FLASH", "warning",
              std::string(kSkillId) + ": flash_thickness_mm " +
              std::to_string(in.flash_thickness_mm) +
              " > 5 mm — excessive flash; revisit die-gutter design");
    } else if (in.flash_thickness_mm > 0.0 && in.flash_thickness_mm < 0.5) {
        r.add("DFM-CDF-FLASH", "info",
              std::string(kSkillId) + ": flash_thickness_mm " +
              std::to_string(in.flash_thickness_mm) +
              " < 0.5 mm — tight trimming; verify press tonnage");
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
        { "shape_id",           in.shape_id },
        { "flash_thickness_mm", in.flash_thickness_mm },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "shape_id",           in.shape_id },
        { "flash_thickness_mm", in.flash_thickness_mm },
        { "geometry_changed",   false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "impression_die";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "H13_tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 20.0;
    tooling.extra = json{
        { "process",            "closed_die_forge" },
        { "shape_id",           in.shape_id },
        { "flash_thickness_mm", in.flash_thickness_mm },
        { "trim_required",      in.flash_thickness_mm > 0.0 },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::closed_die_forge applied: shape_id={} flash={} mm",
                  in.shape_id, in.flash_thickness_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Closed-die identity is a die-shop convention, not a B-rep property.  Pure
// metadata replay.

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

}  // namespace koocadcam::skill::closed_die_forge
