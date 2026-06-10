// @lat: [[engine/skills#camera_island_step]]

#include "camera_island_step.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::camera_island_step {

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

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "camera_island_step: face_id out of range");
    }
    if (!(in.outer_island_w_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "camera_island_step: outer_island_w_mm must be > 0");
    }
    if (!(in.outer_island_h_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "camera_island_step: outer_island_h_mm must be > 0");
    }
    if (!(in.raised_height_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "camera_island_step: raised_height_mm must be > 0");
    }
    if (!(in.inner_lens_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "camera_island_step: inner_lens_dia_mm must be > 0");
    }
    if (!(in.inner_lens_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "camera_island_step: inner_lens_depth_mm must be > 0");
    }
    if (in.inner_lens_dia_mm > 0.0 && in.outer_island_w_mm > 0.0 &&
        in.inner_lens_dia_mm >= in.outer_island_w_mm) {
        r.add("DFM-CAM-LENS", "error",
              "camera_island_step: inner_lens_dia (" +
              std::to_string(in.inner_lens_dia_mm) +
              ") must be < outer_island_w (" +
              std::to_string(in.outer_island_w_mm) + ")");
    }
    if (in.inner_lens_dia_mm > 0.0 && in.outer_island_h_mm > 0.0 &&
        in.inner_lens_dia_mm >= in.outer_island_h_mm) {
        r.add("DFM-CAM-LENS", "error",
              "camera_island_step: inner_lens_dia (" +
              std::to_string(in.inner_lens_dia_mm) +
              ") must be < outer_island_h (" +
              std::to_string(in.outer_island_h_mm) + ")");
    }
    if (in.inner_lens_depth_mm > 0.0 && in.raised_height_mm > 0.0 &&
        in.inner_lens_depth_mm > in.raised_height_mm + 1e-6) {
        r.add("DFM-CAM-LENS", "warning",
              "lens recess depth (" + std::to_string(in.inner_lens_depth_mm) +
              ") > raised step height (" +
              std::to_string(in.raised_height_mm) + ")");
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "camera_island_step DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();
    const double cx    = in.camera_center_x_mm;
    const double cy    = in.camera_center_y_mm;
    constexpr double kEmbed = 0.05;

    const double iW = in.outer_island_w_mm;
    const double iH = in.outer_island_h_mm;
    const double Hs = in.raised_height_mm;
    const double Dl = in.inner_lens_depth_mm;
    const double rL = in.inner_lens_dia_mm / 2.0;

    // Op 1: fuse raised rectangular step onto the face
    TopoDS_Shape working = wp.shape();
    {
        const gp_Pnt origin(cx - iW / 2.0, cy - iH / 2.0, baseZ - kEmbed);
        const gp_Ax2 ax(origin, gp::DZ());
        const TopoDS_Shape step = pr::box(ax, iW, iH, Hs + kEmbed);
        working = pr::fuse(working, step);
    }

    // Op 2: fuse a thin wider "chamfer ledge" base (slightly larger footprint
    // at the bottom, very thin) — simulates the chamfered transition skirt.
    const double ledgeOver  = std::min(0.6, std::max(0.1, Hs * 0.25));
    const double ledgeThick = std::max(0.05, Hs * 0.2);
    {
        const double w = iW + 2.0 * ledgeOver;
        const double h = iH + 2.0 * ledgeOver;
        const gp_Pnt origin(cx - w / 2.0, cy - h / 2.0, baseZ - kEmbed);
        const gp_Ax2 ax(origin, gp::DZ());
        const TopoDS_Shape ledge = pr::box(ax, w, h, ledgeThick + kEmbed);
        working = pr::fuse(working, ledge);
    }

    const double volAfterFuses = volumeOf(working);

    // Op 3: cut the circular lens recess into the raised top
    {
        const gp_Pnt c(cx, cy, baseZ + Hs - Dl);
        const gp_Ax2 ax(c, gp::DZ());
        const TopoDS_Shape lens = pr::cylinder(ax, rL, Dl + kEmbed);
        working = pr::cut(working, lens);
    }

    // Op 4: cut a thin perimeter chamfer-relief box around the step's top
    // edge — implemented as a hollow ring cut (outer box minus inner box).
    const double chamW    = std::min(0.5, std::max(0.05, Hs * 0.2));
    const double chamDep  = std::max(0.05, Hs * 0.15);
    {
        // Outer ring box (slightly larger than step footprint)
        const double outerW = iW + 2.0 * 0.05;
        const double outerH = iH + 2.0 * 0.05;
        const gp_Pnt oOrigin(cx - outerW / 2.0, cy - outerH / 2.0,
                             baseZ + Hs - chamDep);
        const gp_Ax2 oAx(oOrigin, gp::DZ());
        const TopoDS_Shape outerBox = pr::box(oAx, outerW, outerH,
                                              chamDep + kEmbed);
        // Inner block (the region we DON'T want to cut)
        const double innerW = iW - 2.0 * chamW;
        const double innerH = iH - 2.0 * chamW;
        if (innerW > 0.1 && innerH > 0.1) {
            const gp_Pnt iOrigin(cx - innerW / 2.0, cy - innerH / 2.0,
                                 baseZ + Hs - chamDep - kEmbed);
            const gp_Ax2 iAx(iOrigin, gp::DZ());
            const TopoDS_Shape innerBox = pr::box(iAx, innerW, innerH,
                                                  chamDep + 2.0 * kEmbed);
            const TopoDS_Shape ring = pr::cut(outerBox, innerBox);
            working = pr::cut(working, ring);
        }
    }

    const double volBefore = volumeOf(wp.shape());
    const double volAfter  = volumeOf(working);
    const double volAdded  = volAfter - volBefore;

    // Estimates for the volume bookkeeping
    const double vStep   = iW * iH * Hs;
    const double vLedge  = (iW + 2.0 * ledgeOver) * (iH + 2.0 * ledgeOver)
                           * ledgeThick - vStep * (ledgeThick / Hs);
    const double vLens   = M_PI * rL * rL * Dl;

    json params = {
        { "face_id",              in.face_id },
        { "camera_center_x_mm",   in.camera_center_x_mm },
        { "camera_center_y_mm",   in.camera_center_y_mm },
        { "outer_island_w_mm",    in.outer_island_w_mm },
        { "outer_island_h_mm",    in.outer_island_h_mm },
        { "raised_height_mm",     in.raised_height_mm },
        { "inner_lens_dia_mm",    in.inner_lens_dia_mm },
        { "inner_lens_depth_mm",  in.inner_lens_depth_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "is_phone_feature",        true },
        { "subfeature_count",        4 },
        { "has_chamfered_step",      true },
        { "derived_volume_added_mm3",   volAdded },
        { "volume_after_fuses_mm3",     volAfterFuses - volBefore },
        { "estimated_step_volume_mm3",  vStep },
        { "estimated_ledge_volume_mm3", vLedge },
        { "estimated_lens_volume_mm3",  vLens },
        { "outer_island_w_mm",          in.outer_island_w_mm },
        { "outer_island_h_mm",          in.outer_island_h_mm },
        { "raised_height_mm",           in.raised_height_mm },
        { "inner_lens_dia_mm",          in.inner_lens_dia_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "mold_feature";
    tooling.tool_dia_mm       = std::max(1.0, in.inner_lens_dia_mm * 0.5);
    tooling.stock_removed_mm3 = -(volAdded);   // negative of added
    tooling.est_cycle_time_s  = std::abs(volAdded) / 100.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(working, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::camera_island_step: w={} h={} Hs={} lensD={} vol-net={}",
                  iW, iH, Hs, in.inner_lens_dia_mm, volAdded);

    return SkillOutput{ wpNew, sig };
}

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

}  // namespace koocadcam::skill::camera_island_step
