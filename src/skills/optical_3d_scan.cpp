// @lat: [[engine/skills#optical_3d_scan]]

#include "optical_3d_scan.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::optical_3d_scan {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.baseline_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "optical_3d_scan baseline_mm must be > 0 (got " +
              std::to_string(in.baseline_mm) + ")");
    } else if (in.baseline_mm < 20.0) {
        r.add("DFM-OPT-BASE", "warning",
              "optical_3d_scan baseline_mm " +
              std::to_string(in.baseline_mm) +
              " < 20 mm — too short, poor depth resolution");
    }

    if (in.resolution_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "optical_3d_scan resolution_um must be > 0 (got " +
              std::to_string(in.resolution_um) + ")");
    } else if (in.resolution_um > 200.0) {
        r.add("DFM-OPT-RES", "warning",
              "optical_3d_scan resolution_um " +
              std::to_string(in.resolution_um) +
              " > 200 µm — very coarse, likely not metrology-grade");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "optical_3d_scan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "baseline_mm",   in.baseline_mm },
        { "resolution_um", in.resolution_um },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "baseline_mm",         in.baseline_mm },
        { "resolution_um",       in.resolution_um },
        { "metrology_grade",     in.resolution_um <= 50.0 },
        { "inspection_method",   "optical_stereo_triangulation" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "stereo_camera_pair";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "optical_glass";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle: trigger pair + dense stereo match + filtering (~ 10 s typical).
    tooling.est_cycle_time_s  = 10.0;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — passive stereo optical scan, no material removal";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::optical_3d_scan applied: baseline={} mm res={} µm",
                  in.baseline_mm, in.resolution_um);

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

}  // namespace koocadcam::skill::optical_3d_scan
