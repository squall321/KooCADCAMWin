// @lat: [[engine/skills#wood_joinery]]

#include "wood_joinery.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>

namespace koocadcam::skill::wood_joinery {

using nlohmann::json;

namespace {

bool isValidJointType(const std::string& s)
{
    return s == "mortise_tenon" || s == "dovetail" ||
           s == "finger"        || s == "lap";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isValidJointType(in.joint_type)) {
        r.add("DFM-JOINT-TYPE", "error",
              std::string(kSkillId) + ": joint_type '" + in.joint_type +
              "' unknown — expected 'mortise_tenon' | 'dovetail' | 'finger' | 'lap'");
    }
    if (in.length_mm <= 0.0) {
        r.add("DFM-JOINT-LEN", "error",
              std::string(kSkillId) + ": length_mm must be > 0 (got " +
              std::to_string(in.length_mm) + ")");
    }
    if (in.width_mm <= 0.0) {
        r.add("DFM-JOINT-WID", "error",
              std::string(kSkillId) + ": width_mm must be > 0 (got " +
              std::to_string(in.width_mm) + ")");
    }
    if (in.thickness_mm <= 0.0) {
        r.add("DFM-JOINT-THK", "error",
              std::string(kSkillId) + ": thickness_mm must be > 0 (got " +
              std::to_string(in.thickness_mm) + ")");
    }
    if (in.thickness_mm > 0.0 && in.length_mm > 0.0) {
        const double ratio = in.length_mm / in.thickness_mm;
        if (ratio > 5.0) {
            r.add("DFM-JOINT-RATIO", "info",
                  std::string(kSkillId) + ": length/thickness ratio " +
                  std::to_string(ratio) +
                  " > 5 — tenon/pin is slender; risk of cross-grain breakage");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wood_joinery DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "joint_type",   in.joint_type },
        { "length_mm",    in.length_mm },
        { "width_mm",     in.width_mm },
        { "thickness_mm", in.thickness_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "joint_type",       in.joint_type },
        { "length_mm",        in.length_mm },
        { "width_mm",         in.width_mm },
        { "thickness_mm",     in.thickness_mm },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "wood_joinery_fixture";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // metadata-only
    // Assembly time: glue-up + cure proxy; scales with joint count proxy.
    tooling.est_cycle_time_s  = std::max(30.0,
        (in.length_mm + in.width_mm + in.thickness_mm) * 0.5);
    tooling.extra["joint_type"]    = in.joint_type;
    tooling.extra["dfm_findings"]  = findings;
    tooling.extra["process_category"] = "wood_joinery";
    tooling.extra["geometric_no_op"]  = true;
    tooling.extra["machining_constraint"] =
        "Joinery metadata — actual material removal performed by the "
        "subtractive skills (mill_rect_pocket / dovetail_slot / saw_cut).";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::wood_joinery applied: type={} L={} W={} T={}",
                  in.joint_type, in.length_mm, in.width_mm, in.thickness_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Wood joinery leaves no self-evident geometric trace on a single workpiece
// — the mating geometry is recorded by the subtractive skills.  Recognition
// is metadata-only.

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

}  // namespace koocadcam::skill::wood_joinery
