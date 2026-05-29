// @lat: [[engine/skills#cnc_load_unload]]

#include "cnc_load_unload.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::cnc_load_unload {

using nlohmann::json;

namespace {

bool isKnownMethod(const std::string& m)
{
    return m == "robot" || m == "gantry" || m == "manual";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.payload_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": payload_kg must be > 0 (got " +
              std::to_string(in.payload_kg) + ")");
    } else if (in.payload_kg > 50.0) {
        r.add("DFM-CNC-PAYLOAD", "info",
              std::string(kSkillId) + ": payload_kg " +
              std::to_string(in.payload_kg) +
              " exceeds typical gantry envelope (50 kg) — verify lifter spec");
    }

    if (in.cycle_time_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cycle_time_s must be > 0 (got " +
              std::to_string(in.cycle_time_s) + ")");
    } else if (in.cycle_time_s > 60.0) {
        r.add("DFM-CNC-CYCLE", "info",
              std::string(kSkillId) + ": cycle_time_s " +
              std::to_string(in.cycle_time_s) +
              " exceeds typical load cycle (60 s) — bottleneck risk");
    }

    if (!isKnownMethod(in.load_method)) {
        r.add("DFM-CNC-METHOD", "info",
              std::string(kSkillId) + ": load_method '" + in.load_method +
              "' unrecognised — expected robot | gantry | manual");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cnc_load_unload DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "load_method",  in.load_method },
        { "payload_kg",   in.payload_kg },
        { "cycle_time_s", in.cycle_time_s },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "load_method",      in.load_method },
        { "payload_kg",       in.payload_kg },
        { "cycle_time_s",     in.cycle_time_s },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "cell_loader";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.cycle_time_s;
    tooling.extra = {
        { "process",          "cnc_load_unload" },
        { "load_method",      in.load_method },
        { "payload_kg",       in.payload_kg },
        { "process_category", "material_handling" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::cnc_load_unload applied: method={} payload={}kg cycle={}s",
        in.load_method, in.payload_kg, in.cycle_time_s);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric trace on the workpiece.

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

}  // namespace koocadcam::skill::cnc_load_unload
