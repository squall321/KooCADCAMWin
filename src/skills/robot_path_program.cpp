// @lat: [[engine/skills#robot_path_program]]

#include "robot_path_program.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

namespace koocadcam::skill::robot_path_program {

using nlohmann::json;

namespace {

double segmentLength(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double pathLength(const std::vector<std::array<double, 3>>& wps)
{
    double total = 0.0;
    for (size_t i = 1; i < wps.size(); ++i) {
        total += segmentLength(wps[i - 1], wps[i]);
    }
    return total;
}

bool isKnownMotion(const std::string& m)
{
    return m == "PTP" || m == "LIN" || m == "CIRC";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.waypoints.size() < 2) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": waypoints.size() = " +
              std::to_string(in.waypoints.size()) +
              " < 2 — need at least start + end");
    }
    if (in.payload_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": payload_kg must be > 0");
    }
    if (in.speed_mm_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": speed_mm_s must be > 0");
    }

    if (!isKnownMotion(in.motion_type)) {
        r.add("DFM-ROBOT-MOTION", "info",
              std::string(kSkillId) + ": motion_type '" + in.motion_type +
              "' unrecognised — expected PTP | LIN | CIRC");
    }

    if (in.waypoints.size() >= 2) {
        const double L = pathLength(in.waypoints);
        if (L > 50000.0) {
            r.add("DFM-ROBOT-PATH", "info",
                  std::string(kSkillId) + ": total_length_mm " +
                  std::to_string(L) + " > 50000 — verify program is "
                  "intended (cell-scale guard)");
        }
    }

    if (in.speed_mm_s > 2000.0) {
        r.add("DFM-ROBOT-SPEED", "info",
              std::string(kSkillId) + ": speed_mm_s " +
              std::to_string(in.speed_mm_s) +
              " exceeds typical envelope (2000 mm/s)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "robot_path_program DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json wps = json::array();
    for (const auto& w : in.waypoints) {
        wps.push_back({ w[0], w[1], w[2] });
    }
    const double total = pathLength(in.waypoints);
    const int    N     = static_cast<int>(in.waypoints.size());

    json params = {
        { "waypoints",   wps },
        { "motion_type", in.motion_type },
        { "speed_mm_s",  in.speed_mm_s },
        { "payload_kg",  in.payload_kg },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "waypoints",        wps },
        { "waypoint_count",   N },
        { "total_length_mm",  total },
        { "speed_mm_s",       in.speed_mm_s },
        { "payload_kg",       in.payload_kg },
        { "motion_type",      in.motion_type },
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
    // Cycle = path_length / speed + 0.5 s per waypoint for blending overhead.
    tooling.est_cycle_time_s = (in.speed_mm_s > 0.0 ? total / in.speed_mm_s : 0.0) +
                               0.5 * static_cast<double>(N);
    tooling.extra = {
        { "process",          "programmed_robot_path" },
        { "motion_type",      in.motion_type },
        { "waypoint_count",   N },
        { "total_length_mm",  total },
        { "speed_mm_s",       in.speed_mm_s },
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
        "skill::robot_path_program applied: N={} L={}mm speed={}mm/s motion={}",
        N, total, in.speed_mm_s, in.motion_type);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

}  // namespace koocadcam::skill::robot_path_program
