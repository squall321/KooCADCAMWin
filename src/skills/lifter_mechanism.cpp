// @lat: [[engine/skills#lifter_mechanism]]

#include "lifter_mechanism.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::lifter_mechanism {

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
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "lifter_mechanism: face_id out of range");
    }
    if (!(in.lifter_width_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "lifter_mechanism: lifter_width_mm must be > 0");
    }
    if (!(in.lifter_height_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "lifter_mechanism: lifter_height_mm must be > 0");
    }
    if (!(in.lift_stroke_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "lifter_mechanism: lift_stroke_mm must be > 0");
    }
    if (in.lifter_angle_deg < 0.0 || in.lifter_angle_deg > 15.0) {
        r.add("DFM-LIFTER-ANGLE", "error",
              "lifter_mechanism: lifter_angle_deg " +
              std::to_string(in.lifter_angle_deg) +
              " outside [0, 15] deg");
    }
    if (in.lift_stroke_mm > 0.0 && in.lift_stroke_mm < 0.5) {
        r.add("DFM-LIFTER-CLEAR", "error",
              "lifter_mechanism: lift_stroke " +
              std::to_string(in.lift_stroke_mm) +
              " < 0.5 mm clearance minimum");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lifter_mechanism DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Face Z (treat face as roughly horizontal — slice-1 simplification).
    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double faceZ = faceC.Z();

    const double w   = in.lifter_width_mm;
    const double h   = in.lifter_height_mm;
    const double s   = in.lift_stroke_mm;

    // Build axis-aligned box centred at (base_x, base_y, faceZ - s),
    // extending UP by `s` (the stroke = the depth of the lifter envelope
    // cut INTO the mold).  Width is along +X, height along +Y, stroke
    // along +Z.
    constexpr double kOverhang = 0.05;
    const gp_Pnt origin(
        in.base_x_mm - 0.5 * w,
        in.base_y_mm - 0.5 * h,
        faceZ - s - kOverhang);
    const gp_Ax2 ax(origin, gp::DZ(), gp::DX());
    TopoDS_Shape lifterBox = pr::box(ax, w, h, s + 2.0 * kOverhang);

    // Tilt the box by lifter_angle_deg about the Y axis through (base_x,
    // base_y, faceZ) so the channel angles from vertical.
    if (in.lifter_angle_deg > 1e-6) {
        gp_Trsf rot;
        const gp_Ax1 pivot(gp_Pnt(in.base_x_mm, in.base_y_mm, faceZ),
                           gp::DY());
        rot.SetRotation(pivot, in.lifter_angle_deg * M_PI / 180.0);
        BRepBuilderAPI_Transform xf(lifterBox, rot, /*copy*/ true);
        if (!xf.IsDone()) {
            throw SkillError("lifter_mechanism: tilt transform failed");
        }
        lifterBox = xf.Shape();
    }

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::cut(wp.shape(), lifterBox);
    if (newShape.IsNull()) {
        throw SkillError("lifter_mechanism: cut returned null shape");
    }
    const double volAfter  = volumeOf(newShape);
    const double removed   = volBefore - volAfter;

    json params = {
        { "face_id",          in.face_id },
        { "base_x_mm",        in.base_x_mm },
        { "base_y_mm",        in.base_y_mm },
        { "lifter_angle_deg", in.lifter_angle_deg },
        { "lifter_width_mm",  in.lifter_width_mm },
        { "lifter_height_mm", in.lifter_height_mm },
        { "lift_stroke_mm",   in.lift_stroke_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "mold_feature_type",       "lifter_channel" },
        { "derived_volume_removed",  removed },
        { "tilt_angle_deg",          in.lifter_angle_deg },
        { "volume_before_mm3",       volBefore },
        { "volume_after_mm3",        volAfter },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "mold_feature";
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::lifter_mechanism applied: w={} h={} stroke={} angle={} removed={}",
                  w, h, s, in.lifter_angle_deg, removed);

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
        rf.confidence       = 0.92;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::lifter_mechanism
