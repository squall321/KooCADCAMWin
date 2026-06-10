// @lat: [[engine/skills#scale_body_uniform]]

#include "scale_body_uniform.hpp"

#include "Workpiece.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::scale_body_uniform {

using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (!(in.scale_factor >= 0.5 - 1e-9 && in.scale_factor <= 3.0 + 1e-9)) {
        r.add("DFM-SCALE-RANGE", "error",
              "scale_body_uniform: scale_factor " +
              std::to_string(in.scale_factor) + " outside [0.5, 3.0]");
    }
    if (std::abs(in.scale_factor) < 1e-9) {
        r.add("DFM-INPUT", "error",
              "scale_body_uniform: scale_factor near zero collapses geometry");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "scale_body_uniform DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt center(in.center_xyz[0], in.center_xyz[1], in.center_xyz[2]);
    gp_Trsf trsf;
    trsf.SetScale(center, in.scale_factor);

    BRepBuilderAPI_Transform xform(wp.shape(), trsf, /*copy=*/true);
    if (!xform.IsDone())
        throw SkillError("scale_body_uniform: BRepBuilderAPI_Transform failed");
    const TopoDS_Shape newShape = xform.Shape();

    const double volBefore = volumeOf(wp.shape());
    const double volAfter  = volumeOf(newShape);
    const double expected  = volBefore *
                             in.scale_factor * in.scale_factor * in.scale_factor;

    json params = {
        { "center_xyz",   in.center_xyz },
        { "scale_factor", in.scale_factor },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "parameters",            params },
        { "derived_volume",        volAfter },
        { "volume_before_mm3",     volBefore },
        { "volume_after_mm3",      volAfter },
        { "expected_volume_mm3",   expected },
        { "volume_ratio",          (volBefore > 1e-9 ? volAfter / volBefore : 0.0) },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "body_scale";
    tooling.stock_removed_mm3 = -(volAfter - volBefore);
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::scale_body_uniform: factor={} vol {}→{}",
                  in.scale_factor, volBefore, volAfter);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay is the only reliable path — pure-geometric "is this scaled?"
// requires comparing to an original that we no longer have.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.95;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::scale_body_uniform
