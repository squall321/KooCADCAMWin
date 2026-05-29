// @lat: [[engine/skills#crane_lift]]

#include "crane_lift.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::crane_lift {

using nlohmann::json;

namespace {

bool isKnownClass(const std::string& c)
{
    return c == "light" || c == "standard" || c == "heavy" || c == "ultra";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.lifting_capacity_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": lifting_capacity_kg must be > 0 (got " +
              std::to_string(in.lifting_capacity_kg) + ")");
    } else if (in.lifting_capacity_kg > 50000.0) {
        r.add("DFM-CRANE-CAPACITY", "info",
              std::string(kSkillId) + ": lifting_capacity_kg " +
              std::to_string(in.lifting_capacity_kg) +
              " exceeds typical overhead crane (50 000 kg) — verify rating");
    }

    if (in.lift_height_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": lift_height_m must be > 0 (got " +
              std::to_string(in.lift_height_m) + ")");
    } else if (in.lift_height_m > 25.0) {
        r.add("DFM-CRANE-HEIGHT", "info",
              std::string(kSkillId) + ": lift_height_m " +
              std::to_string(in.lift_height_m) +
              " exceeds typical bay headroom (25 m) — verify hook path");
    }

    if (!isKnownClass(in.lift_class)) {
        r.add("DFM-CRANE-CLASS", "info",
              std::string(kSkillId) + ": lift_class '" + in.lift_class +
              "' unrecognised — expected light | standard | heavy | ultra");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "crane_lift DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "lifting_capacity_kg", in.lifting_capacity_kg },
        { "lift_height_m",       in.lift_height_m },
        { "lift_class",          in.lift_class },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "lifting_capacity_kg", in.lifting_capacity_kg },
        { "lift_height_m",       in.lift_height_m },
        { "lift_class",          in.lift_class },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "overhead_crane";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rated hoisting speed ~0.5 m/s + 10 s rig/release per direction.
    tooling.est_cycle_time_s  = 20.0 + (in.lift_height_m / 0.5) * 2.0;
    tooling.extra = {
        { "process",             "crane_lift" },
        { "lifting_capacity_kg", in.lifting_capacity_kg },
        { "lift_height_m",       in.lift_height_m },
        { "lift_class",          in.lift_class },
        { "process_category",    "material_handling" },
        { "geometric_no_op",     true },
        { "dfm_findings",        findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::crane_lift applied: capacity={}kg height={}m class={}",
        in.lifting_capacity_kg, in.lift_height_m, in.lift_class);

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

}  // namespace koocadcam::skill::crane_lift
