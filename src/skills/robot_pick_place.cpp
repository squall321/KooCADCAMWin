// @lat: [[engine/skills#robot_pick_place]]

#include "robot_pick_place.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

namespace koocadcam::skill::robot_pick_place {

using nlohmann::json;

namespace {

double dist3(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool isKnownGripper(const std::string& g)
{
    return g == "parallel_jaw" || g == "vacuum" ||
           g == "magnetic"     || g == "soft";
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
    } else if (in.payload_kg > 25.0) {
        r.add("DFM-ROBOT-PAYLOAD", "info",
              std::string(kSkillId) + ": payload_kg " +
              std::to_string(in.payload_kg) +
              " exceeds typical 6-axis robot envelope (25 kg) — "
              "select a heavy-payload arm");
    }

    const double travel = dist3(in.source_xyz, in.target_xyz);
    if (travel < 0.01) {
        r.add("DFM-ROBOT-DEGENERATE", "warning",
              std::string(kSkillId) + ": source_xyz == target_xyz "
              "(travel " + std::to_string(travel) + " mm) — no-motion cycle");
    } else if (travel > 2500.0) {
        r.add("DFM-ROBOT-REACH", "info",
              std::string(kSkillId) + ": travel " + std::to_string(travel) +
              " mm exceeds typical reach envelope (2500 mm) — verify arm size");
    }

    if (!isKnownGripper(in.gripper_type)) {
        r.add("DFM-ROBOT-GRIPPER", "info",
              std::string(kSkillId) + ": gripper_type '" + in.gripper_type +
              "' unrecognised — expected parallel_jaw | vacuum | magnetic | soft");
    }

    if (in.approach_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": approach_mm must be ≥ 0");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "robot_pick_place DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double travel = dist3(in.source_xyz, in.target_xyz);

    json src = { in.source_xyz[0], in.source_xyz[1], in.source_xyz[2] };
    json tgt = { in.target_xyz[0], in.target_xyz[1], in.target_xyz[2] };

    json params = {
        { "source_xyz",   src },
        { "target_xyz",   tgt },
        { "gripper_type", in.gripper_type },
        { "payload_kg",   in.payload_kg },
        { "approach_mm",  in.approach_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "source_xyz",       src },
        { "target_xyz",       tgt },
        { "gripper_type",     in.gripper_type },
        { "payload_kg",       in.payload_kg },
        { "approach_mm",      in.approach_mm },
        { "travel_mm",        travel },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "robot_arm";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Approach (50 mm) + 250 mm/s linear travel + retract.  Add 2 s for grip/release.
    tooling.est_cycle_time_s = 2.0 + (in.approach_mm * 2.0 + travel) / 250.0;
    tooling.extra = {
        { "process",          "robotic_pick_and_place" },
        { "gripper_type",     in.gripper_type },
        { "payload_kg",       in.payload_kg },
        { "travel_mm",        travel },
        { "process_category", "material_handling" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::robot_pick_place applied: gripper={} payload={}kg travel={}mm",
        in.gripper_type, in.payload_kg, travel);

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

}  // namespace koocadcam::skill::robot_pick_place
