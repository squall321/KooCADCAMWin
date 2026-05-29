// @lat: [[engine/skills#frame_assemble]]

#include "frame_assemble.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::frame_assemble {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.member_count < 2) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": member_count must be ≥ 2 (got " +
              std::to_string(in.member_count) + ") — frame requires ≥ 2 members");
    }

    if (in.joint_count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": joint_count must be ≥ 1 (got " +
              std::to_string(in.joint_count) + ")");
    }

    if (in.frame_type != "steel" && in.frame_type != "wood") {
        r.add("DFM-FRAME-TYPE", "error",
              std::string(kSkillId) + ": frame_type '" + in.frame_type +
              "' not in {steel, wood}");
    }

    if (in.member_count >= 2 && in.joint_count >= 1) {
        if (in.joint_count > 4 * in.member_count) {
            r.add("DFM-FRAME-JOINT", "info",
                  std::string(kSkillId) + ": joint_count " +
                  std::to_string(in.joint_count) +
                  " > 4 × member_count — unusually high connection density, verify");
        }
        if (in.joint_count < in.member_count - 1) {
            r.add("DFM-FRAME-JOINT", "info",
                  std::string(kSkillId) + ": joint_count " +
                  std::to_string(in.joint_count) +
                  " < (member_count − 1) — frame may be disconnected");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "frame_assemble DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double joints_per_member =
        in.member_count > 0
            ? static_cast<double>(in.joint_count) / static_cast<double>(in.member_count)
            : 0.0;

    json params = {
        { "frame_type",   in.frame_type },
        { "member_count", in.member_count },
        { "joint_count",  in.joint_count },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "frame_type",          in.frame_type },
        { "member_count",        in.member_count },
        { "joint_count",         in.joint_count },
        { "joints_per_member",   joints_per_member },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    // Steel: torque wrench + impact for high-strength bolts; wood: nail gun.
    tooling.tool_type         = (in.frame_type == "steel") ? "torque_wrench" : "framing_nailer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (in.frame_type == "steel") ? "hardened_steel" : "carbon_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Steel joint: ~ 5 min/joint (rigging + torque).  Wood: ~ 30 s/joint.
    const double per_joint_s = (in.frame_type == "steel") ? 300.0 : 30.0;
    tooling.est_cycle_time_s  = per_joint_s * static_cast<double>(in.joint_count);
    tooling.extra = {
        { "process",            "structural_frame_erection" },
        { "frame_type",         in.frame_type },
        { "member_count",       in.member_count },
        { "joint_count",        in.joint_count },
        { "joints_per_member",  joints_per_member },
        { "process_category",   "construction_assembly" },
        { "geometric_no_op",    true },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::frame_assemble applied: type={} members={} joints={}",
        in.frame_type, in.member_count, in.joint_count);

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

}  // namespace koocadcam::skill::frame_assemble
