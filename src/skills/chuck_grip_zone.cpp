// @lat: [[engine/skills#chuck_grip_zone]]

#include "chuck_grip_zone.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::chuck_grip_zone {

using nlohmann::json;

namespace {

bool dirIsValid(const gp_Dir& d)
{
    const double m = std::sqrt(d.X() * d.X() + d.Y() * d.Y() + d.Z() * d.Z());
    // gp_Dir is unit-normalised on construction; this is a defensive check
    // for NaN / Inf produced by upstream calculation.
    return std::isfinite(m) && std::abs(m - 1.0) < 1e-6;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.grip_length <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": grip_length must be > 0 (got " +
              std::to_string(in.grip_length) + ")");
    } else if (in.grip_length < 3.0) {
        r.add("CG-LENGTH-SHORT", "warning",
              std::string(kSkillId) + ": grip_length " +
              std::to_string(in.grip_length) +
              " mm < 3 mm — chuck jaw engagement may be inadequate");
    }
    if (!dirIsValid(in.cylinder_axis.Direction())) {
        r.add("CG-AXIS-NORM", "error",
              std::string(kSkillId) + ": cylinder_axis.Direction must be a unit vector");
    }
    (void)wp;
    return r;
}

// ── Synthesis (metadata-only) ────────────────────────────────────────────

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

    const gp_Pnt o = in.cylinder_axis.Location();
    const gp_Dir d = in.cylinder_axis.Direction();

    json axis = {
        { "origin",    { o.X(), o.Y(), o.Z() } },
        { "direction", { d.X(), d.Y(), d.Z() } },
    };
    json params = {
        { "cylinder_axis", axis },
        { "grip_length",   in.grip_length },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "cylinder_axis",     axis },
        { "grip_length",       in.grip_length },
        { "geometry_changed",  false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "chuck";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "hardened_steel_jaws";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 20.0;        // operator load + jaw clamp
    tooling.extra["grip_length"]      = in.grip_length;
    tooling.extra["geometric_no_op"]  = true;
    tooling.extra["protected_zone"]   = true;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::chuck_grip_zone applied: grip_length={} mm", in.grip_length);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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

}  // namespace koocadcam::skill::chuck_grip_zone
