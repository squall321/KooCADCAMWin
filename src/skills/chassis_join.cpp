// @lat: [[engine/skills#chassis_join]]

#include "chassis_join.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::chassis_join {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.weld_count < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": weld_count must be >= 0 (got " +
              std::to_string(in.weld_count) + ")");
    }
    if (in.sealer_meters < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sealer_meters must be >= 0 (got " +
              std::to_string(in.sealer_meters) + ")");
    }
    if (in.bolt_count < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": bolt_count must be >= 0 (got " +
              std::to_string(in.bolt_count) + ")");
    }
    if (in.weld_count + in.bolt_count <= 0) {
        r.add("DFM-CHASSIS-EMPTY", "error",
              std::string(kSkillId) +
              ": weld_count + bolt_count == 0 — no actual join performed");
    }
    if (in.weld_count > 200) {
        r.add("DFM-CHASSIS-WELD", "info",
              std::string(kSkillId) + ": weld_count " +
              std::to_string(in.weld_count) +
              " > 200 — verify gun reach / cycle-time budget");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "chassis_join DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "weld_count",    in.weld_count },
        { "sealer_meters", in.sealer_meters },
        { "bolt_count",    in.bolt_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "weld_count",       in.weld_count },
        { "sealer_meters",    in.sealer_meters },
        { "bolt_count",       in.bolt_count },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "chassis_join_station";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // 2 s / spot weld + 4 s/m sealer + 6 s / bolt + 20 s station handshake.
    tooling.est_cycle_time_s  = 20.0
                              + 2.0 * static_cast<double>(in.weld_count)
                              + 4.0 * in.sealer_meters
                              + 6.0 * static_cast<double>(in.bolt_count);
    tooling.extra = {
        { "process",          "chassis_join" },
        { "weld_count",       in.weld_count },
        { "sealer_meters",    in.sealer_meters },
        { "bolt_count",       in.bolt_count },
        { "process_category", "automotive_assembly" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::chassis_join applied: welds={} sealer={}m bolts={}",
        in.weld_count, in.sealer_meters, in.bolt_count);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only — no geometric fingerprint.

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

}  // namespace koocadcam::skill::chassis_join
