// @lat: [[engine/skills#mirror_body_about_plane]]

#include "mirror_body_about_plane.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::mirror_body_about_plane {

namespace pr = koocadcam::engine::prim;
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
    const double n2 = in.plane_normal[0] * in.plane_normal[0] +
                      in.plane_normal[1] * in.plane_normal[1] +
                      in.plane_normal[2] * in.plane_normal[2];
    if (n2 < 1e-18) {
        r.add("DFM-INPUT", "error",
              "mirror_body_about_plane: plane_normal is zero-length");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mirror_body_about_plane DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double nnorm = std::sqrt(in.plane_normal[0] * in.plane_normal[0] +
                                   in.plane_normal[1] * in.plane_normal[1] +
                                   in.plane_normal[2] * in.plane_normal[2]);
    const gp_Pnt origin(in.plane_origin[0],
                        in.plane_origin[1],
                        in.plane_origin[2]);
    const gp_Dir normal(in.plane_normal[0] / nnorm,
                        in.plane_normal[1] / nnorm,
                        in.plane_normal[2] / nnorm);

    gp_Trsf mirror;
    mirror.SetMirror(gp_Ax2(origin, normal));

    BRepBuilderAPI_Transform xform(wp.shape(), mirror, /*copy=*/true);
    if (!xform.IsDone())
        throw SkillError(
            "mirror_body_about_plane: BRepBuilderAPI_Transform failed");
    const TopoDS_Shape mirrored = xform.Shape();

    const double volBefore = volumeOf(wp.shape());

    // Fuse original + mirrored.  Fall back to mirrored alone if fuse fails
    // (e.g. for non-overlapping disjoint cases — we still want a result).
    TopoDS_Shape newShape;
    bool fusedOk = false;
    try {
        newShape = pr::fuse(wp.shape(), mirrored);
        fusedOk  = !newShape.IsNull();
    } catch (const Standard_Failure& ex) {
        spdlog::debug("mirror_body_about_plane: fuse threw — {}",
                      ex.what());
    }
    if (!fusedOk) {
        // Fallback: keep the mirrored shape only so downstream pipelines do
        // not crash on a null result.
        newShape = mirrored;
    }

    const double volAfter = volumeOf(newShape);

    json params = {
        { "plane_origin", in.plane_origin },
        { "plane_normal", in.plane_normal },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "parameters",          params },
        { "derived_volume",      volAfter },
        { "volume_before_mm3",   volBefore },
        { "volume_after_mm3",    volAfter },
        { "fused",               fusedOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "body_mirror";
    tooling.stock_removed_mm3 = -(volAfter - volBefore);   // added material
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::mirror_body_about_plane: vol {}→{} fused={}",
                  volBefore, volAfter, fusedOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

}  // namespace koocadcam::skill::mirror_body_about_plane
