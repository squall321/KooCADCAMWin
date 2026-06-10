// @lat: [[engine/skills#copy_body_with_transform]]

#include "copy_body_with_transform.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::copy_body_with_transform {

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
    const double an2 = in.rotation_axis_dir[0] * in.rotation_axis_dir[0] +
                       in.rotation_axis_dir[1] * in.rotation_axis_dir[1] +
                       in.rotation_axis_dir[2] * in.rotation_axis_dir[2];
    if (an2 < 1e-18) {
        r.add("DFM-INPUT", "error",
              "copy_body_with_transform: rotation_axis_dir is zero-length");
    }
    const double tnorm2 = in.translation_xyz[0] * in.translation_xyz[0] +
                          in.translation_xyz[1] * in.translation_xyz[1] +
                          in.translation_xyz[2] * in.translation_xyz[2];
    if (tnorm2 < 1e-18 && std::abs(in.rotation_angle_deg) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "copy_body_with_transform: transform is identity (translation=0 "
              "AND rotation_angle=0)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "copy_body_with_transform DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Build the rotation about axis(origin, dir) by angle (radians).
    gp_Trsf rot;
    const double anorm = std::sqrt(
        in.rotation_axis_dir[0] * in.rotation_axis_dir[0] +
        in.rotation_axis_dir[1] * in.rotation_axis_dir[1] +
        in.rotation_axis_dir[2] * in.rotation_axis_dir[2]);
    const gp_Dir axisDir(in.rotation_axis_dir[0] / anorm,
                         in.rotation_axis_dir[1] / anorm,
                         in.rotation_axis_dir[2] / anorm);
    const gp_Pnt axisOrigin(in.rotation_axis_origin[0],
                            in.rotation_axis_origin[1],
                            in.rotation_axis_origin[2]);
    const gp_Ax1 rotAxis(axisOrigin, axisDir);
    const double angleRad = in.rotation_angle_deg * M_PI / 180.0;
    rot.SetRotation(rotAxis, angleRad);

    gp_Trsf trans;
    trans.SetTranslation(gp_Vec(in.translation_xyz[0],
                                in.translation_xyz[1],
                                in.translation_xyz[2]));

    // Combined T = trans * rot  (rotate first, then translate).
    gp_Trsf combined = trans * rot;

    BRepBuilderAPI_Transform xform(wp.shape(), combined, true /* copy */);
    if (!xform.IsDone())
        throw SkillError("copy_body_with_transform: BRepBuilderAPI_Transform failed");
    const TopoDS_Shape copyShape = xform.Shape();

    // Pack { original, copy } into a compound.
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    builder.Add(compound, wp.shape());
    builder.Add(compound, copyShape);

    // Count solids in the result.
    int solidCount = 0;
    for (TopExp_Explorer e(compound, TopAbs_SOLID); e.More(); e.Next()) ++solidCount;

    const double volOriginal = volumeOf(wp.shape());
    const double volCopy     = volumeOf(copyShape);

    json params = {
        { "translation_xyz",      in.translation_xyz },
        { "rotation_axis_origin", in.rotation_axis_origin },
        { "rotation_axis_dir",    in.rotation_axis_dir },
        { "rotation_angle_deg",   in.rotation_angle_deg },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "translation_xyz",       in.translation_xyz },
        { "rotation_axis_origin",  in.rotation_axis_origin },
        { "rotation_axis_dir",     { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "rotation_angle_deg",    in.rotation_angle_deg },
        { "solid_count",           solidCount },
        { "derived_original_volume_mm3", volOriginal },
        { "derived_copy_volume_mm3",     volCopy },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "robotic_pick_place";
    tooling.stock_removed_mm3 = -volCopy;     // "adds" a copy of mass
    tooling.est_cycle_time_s  = 2.0;

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(compound, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::copy_body_with_transform: solid_count={} vol original={} copy={}",
                  solidCount, volOriginal, volCopy);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: a TopoDS_Compound containing ≥ 2 sub-solids whose
// volumes are nearly equal (within 0.1%) is a strong "duplicate body"
// signal.  We surface that candidate plus any history replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    if (shape.IsNull()) return out;

    // History replay (high confidence).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.94;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Pure-geometric fallback: detect compound with ≥2 solids of equal vol.
    if (shape.ShapeType() != TopAbs_COMPOUND) return out;
    std::vector<double> vols;
    for (TopExp_Explorer e(shape, TopAbs_SOLID); e.More(); e.Next()) {
        GProp_GProps p;
        BRepGProp::VolumeProperties(e.Current(), p);
        vols.push_back(p.Mass());
    }
    if (vols.size() < 2) return out;
    for (size_t i = 0; i < vols.size(); ++i) {
        for (size_t j = i + 1; j < vols.size(); ++j) {
            const double a = vols[i];
            const double b = vols[j];
            const double mx = std::max(std::abs(a), std::abs(b));
            if (mx < 1e-9) continue;
            if (std::abs(a - b) / mx > 1e-3) continue;
            RecognizedFeature rf;
            rf.skill_id = kSkillId;
            rf.recovered_params = {
                { "translation_xyz",      { 0.0, 0.0, 0.0 } },
                { "rotation_axis_origin", { 0.0, 0.0, 0.0 } },
                { "rotation_axis_dir",    { 0.0, 0.0, 1.0 } },
                { "rotation_angle_deg",   0.0 },
            };
            rf.confidence = 0.55;
            rf.matched_geometry = {
                { "solid_count",   static_cast<int>(vols.size()) },
                { "volume_a_mm3",  a },
                { "volume_b_mm3",  b },
            };
            out.push_back(rf);
            return out;   // one geometric match is enough
        }
    }
    return out;
}

}  // namespace koocadcam::skill::copy_body_with_transform
