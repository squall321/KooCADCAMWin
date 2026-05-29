// @lat: [[engine/skills#hoist_op]]

#include "hoist_op.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::hoist_op {

using nlohmann::json;

namespace {

bool isKnownType(const std::string& t)
{
    return t == "chain" || t == "wire_rope" || t == "lever";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.hoist_capacity_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": hoist_capacity_kg must be > 0 (got " +
              std::to_string(in.hoist_capacity_kg) + ")");
    } else if (in.hoist_capacity_kg > 5000.0) {
        r.add("DFM-HOIST-CAPACITY", "info",
              std::string(kSkillId) + ": hoist_capacity_kg " +
              std::to_string(in.hoist_capacity_kg) +
              " exceeds typical jib/monorail hoist (5000 kg) — escalate to crane");
    }

    if (in.lift_height_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": lift_height_m must be > 0 (got " +
              std::to_string(in.lift_height_m) + ")");
    } else if (in.lift_height_m > 10.0) {
        r.add("DFM-HOIST-HEIGHT", "info",
              std::string(kSkillId) + ": lift_height_m " +
              std::to_string(in.lift_height_m) +
              " exceeds typical hoist stroke (10 m) — verify chain/rope length");
    }

    if (!isKnownType(in.hoist_type)) {
        r.add("DFM-HOIST-TYPE", "info",
              std::string(kSkillId) + ": hoist_type '" + in.hoist_type +
              "' unrecognised — expected chain | wire_rope | lever");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hoist_op DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "hoist_type",        in.hoist_type },
        { "hoist_capacity_kg", in.hoist_capacity_kg },
        { "lift_height_m",     in.lift_height_m },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "hoist_type",        in.hoist_type },
        { "hoist_capacity_kg", in.hoist_capacity_kg },
        { "lift_height_m",     in.lift_height_m },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "hoist";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Typical hoist speed ~0.3 m/s + 8 s rig/release per direction.
    tooling.est_cycle_time_s  = 16.0 + (in.lift_height_m / 0.3) * 2.0;
    tooling.extra = {
        { "process",           "hoist_op" },
        { "hoist_type",        in.hoist_type },
        { "hoist_capacity_kg", in.hoist_capacity_kg },
        { "lift_height_m",     in.lift_height_m },
        { "process_category",  "material_handling" },
        { "geometric_no_op",   true },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::hoist_op applied: type={} capacity={}kg height={}m",
        in.hoist_type, in.hoist_capacity_kg, in.lift_height_m);

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

}  // namespace koocadcam::skill::hoist_op
