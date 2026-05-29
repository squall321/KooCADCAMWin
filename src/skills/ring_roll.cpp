// @lat: [[engine/skills#ring_roll]]

#include "ring_roll.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::ring_roll {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_id_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": target_id_mm must be > 0 (got " +
              std::to_string(in.target_id_mm) + ")");
    }
    if (in.target_od_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": target_od_mm must be > 0 (got " +
              std::to_string(in.target_od_mm) + ")");
    }
    if (in.target_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": target_height_mm must be > 0 (got " +
              std::to_string(in.target_height_mm) + ")");
    }

    if (in.target_id_mm > 0.0 && in.target_od_mm > 0.0 &&
        in.target_od_mm <= in.target_id_mm)
    {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": target_od_mm " +
              std::to_string(in.target_od_mm) +
              " must be > target_id_mm " + std::to_string(in.target_id_mm));
    }

    if (in.target_id_mm > 0.0 && in.target_od_mm > in.target_id_mm) {
        const double ratio = in.target_id_mm / in.target_od_mm;
        if (ratio > 0.95) {
            r.add("DFM-RR-RATIO", "error",
                  std::string(kSkillId) + ": id_od_ratio " +
                  std::to_string(ratio) +
                  " > 0.95 — wall too thin; rolling collapse risk");
        } else if (ratio < 0.50) {
            r.add("DFM-RR-RATIO", "info",
                  std::string(kSkillId) + ": id_od_ratio " +
                  std::to_string(ratio) +
                  " < 0.50 — very thick wall; multiple passes likely needed");
        }
    }

    if (in.target_height_mm > 0.0 && in.target_od_mm > 0.0 &&
        in.target_height_mm > in.target_od_mm)
    {
        r.add("DFM-RR-HEIGHT", "warning",
              std::string(kSkillId) + ": height " +
              std::to_string(in.target_height_mm) +
              " > OD " + std::to_string(in.target_od_mm) +
              " — column ring; axial-roll tip risk");
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

    const double ratio = (in.target_od_mm > 0.0)
        ? in.target_id_mm / in.target_od_mm : 0.0;

    json params = {
        { "target_id_mm",     in.target_id_mm },
        { "target_od_mm",     in.target_od_mm },
        { "target_height_mm", in.target_height_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "target_id_mm",     in.target_id_mm },
        { "target_od_mm",     in.target_od_mm },
        { "target_height_mm", in.target_height_mm },
        { "id_od_ratio",      ratio },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "ring_rolling_mill";
    tooling.tool_dia_mm       = in.target_od_mm;
    tooling.tool_length_mm    = in.target_height_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 60.0 + in.target_od_mm * 0.5;
    tooling.extra = json{
        { "process",          "ring_roll" },
        { "target_id_mm",     in.target_id_mm },
        { "target_od_mm",     in.target_od_mm },
        { "target_height_mm", in.target_height_mm },
        { "id_od_ratio",      ratio },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ring_roll applied: ID={} OD={} H={} ratio={}",
                  in.target_id_mm, in.target_od_mm,
                  in.target_height_mm, ratio);

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

}  // namespace koocadcam::skill::ring_roll
