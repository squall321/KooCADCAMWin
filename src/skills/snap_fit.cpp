// @lat: [[engine/skills#snap_fit]]

#include "snap_fit.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::snap_fit {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wall_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "snap_fit wall_thickness_mm must be > 0");
    } else if (in.wall_thickness_mm < 0.8) {
        r.add("DFM-SNAP-WALL", "error",
              "snap_fit wall_thickness " + std::to_string(in.wall_thickness_mm) +
              " mm < 0.8 mm minimum");
    }

    if (in.length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "snap_fit length_mm must be > 0");
    }
    if (in.width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "snap_fit width_mm must be > 0");
    }
    if (in.hook_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "snap_fit hook_height_mm must be > 0");
    }
    if (in.hook_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "snap_fit hook_depth_mm must be > 0");
    }

    if (in.hook_depth_mm > in.hook_height_mm + 1e-9) {
        r.add("DFM-SNAP-HOOK", "error",
              "snap_fit hook_depth " + std::to_string(in.hook_depth_mm) +
              " > hook_height " + std::to_string(in.hook_height_mm) +
              " — engagement geometry invalid");
    }

    if (in.wall_thickness_mm > 0.0 && in.length_mm < in.wall_thickness_mm * 5.0) {
        r.add("DFM-SNAP-LENGTH", "warning",
              "snap_fit length/wall_thickness ratio " +
              std::to_string(in.length_mm / in.wall_thickness_mm) +
              " < 5 — beam may not deflect enough to snap");
    }

    // direction_xy must be in XY plane.
    if (std::abs(in.direction_xy.Z()) > 1e-3) {
        r.add("DFM-INPUT", "error",
              "snap_fit direction_xy must be in XY plane (Z component must be 0)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "snap_fit DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("snap_fit: entry_face datum unresolved");

    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("snap_fit: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();

    // For slice-1, restrict to faces whose normal is ±Z (snap_fit on top/bot).
    if (std::abs(std::abs(outward.Z()) - 1.0) > 1e-2) {
        throw SkillError("snap_fit: slice-1 supports only ±Z entry_face normals");
    }

    // Local frame:
    //   xLoc = direction_xy (along the beam length)
    //   yLoc = perpendicular to xLoc in the entry plane (width direction)
    //   zLoc = outward (extrusion direction, ±Z here)
    gp_Vec xVec(in.direction_xy);
    if (xVec.Magnitude() < 1e-9) {
        throw SkillError("snap_fit: direction_xy has zero magnitude");
    }
    xVec.Normalize();
    // yLoc = outward × xVec
    gp_Vec yVec = gp_Vec(outward).Crossed(xVec);
    if (yVec.Magnitude() < 1e-9) {
        throw SkillError("snap_fit: direction_xy parallel to entry-face normal");
    }
    yVec.Normalize();

    const gp_Dir xLoc(xVec.X(), xVec.Y(), xVec.Z());
    const gp_Dir yLoc(yVec.X(), yVec.Y(), yVec.Z());

    // Project start point onto entry plane.
    auto projectOntoPlane = [&](double x, double y) {
        const gp_Pnt P0 = entryPlane.Location();
        const gp_Dir n  = entryPlane.Axis().Direction();
        if (std::abs(n.Z()) < 1e-9) return gp_Pnt(x, y, P0.Z());
        const double z = P0.Z() - (n.X() * (x - P0.X()) + n.Y() * (y - P0.Y())) / n.Z();
        return gp_Pnt(x, y, z);
    };
    const gp_Pnt startPnt = projectOntoPlane(in.start_x_mm, in.start_y_mm);

    // --- Beam box ----------------------------------------------------------
    //
    // We want a box of (length × width × wall_thickness) along (xLoc, yLoc,
    // outward).  Box origin (the corner with smallest local x,y,z) is at
    // startPnt - (width/2) yLoc.
    //
    // gp_Ax2(P, V, Vx) interprets V as local-Z and Vx as local-X.  We pick
    // V = outward, Vx = xLoc so DZ = wall_thickness along outward, DX =
    // length along xLoc, DY = width along yLoc (= V × Vx).
    //
    // Push the box slightly INTO the workpiece by `kEmbed` to guarantee a
    // clean Boolean fuse (avoids coplanar-face issues).
    constexpr double kEmbed = 0.02;
    const gp_Pnt beamOrigin(
        startPnt.X() - (in.width_mm / 2.0) * yLoc.X() - kEmbed * outward.X(),
        startPnt.Y() - (in.width_mm / 2.0) * yLoc.Y() - kEmbed * outward.Y(),
        startPnt.Z() - (in.width_mm / 2.0) * yLoc.Z() - kEmbed * outward.Z());
    const gp_Ax2 beamAx(beamOrigin, outward, xLoc);
    const TopoDS_Shape beam = pr::box(beamAx, in.length_mm,
                                      in.width_mm, in.wall_thickness_mm + kEmbed);

    // --- Hook lip box ------------------------------------------------------
    //
    // Sits on top of the beam at its far end.  Footprint = hook_depth ×
    // width; extruded along outward by hook_height; positioned such that the
    // BACK of the hook (closer to startPnt) is at local x = length (i.e. the
    // hook extends from local x = length to local x = length + hook_depth).
    // Its bottom is at local z = wall_thickness (on top of the beam).
    // Hook origin slightly EMBEDS into the beam top in -outward by kEmbed
    // and back into the beam end in -xLoc by kEmbed so the fuse is clean.
    const gp_Pnt hookOrigin(
        startPnt.X() + (in.length_mm - kEmbed) * xLoc.X()
                     - (in.width_mm / 2.0) * yLoc.X()
                     + (in.wall_thickness_mm - kEmbed) * outward.X(),
        startPnt.Y() + (in.length_mm - kEmbed) * xLoc.Y()
                     - (in.width_mm / 2.0) * yLoc.Y()
                     + (in.wall_thickness_mm - kEmbed) * outward.Y(),
        startPnt.Z() + (in.length_mm - kEmbed) * xLoc.Z()
                     - (in.width_mm / 2.0) * yLoc.Z()
                     + (in.wall_thickness_mm - kEmbed) * outward.Z());
    const gp_Ax2 hookAx(hookOrigin, outward, xLoc);
    const TopoDS_Shape hook = pr::box(hookAx, in.hook_depth_mm + kEmbed,
                                       in.width_mm, in.hook_height_mm + kEmbed);

    // Fuse beam + hook + workpiece.
    const TopoDS_Shape lShape = pr::fuseMany(beam, { hook });
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), lShape);

    json params = {
        { "entry_face_kind",  "resolved_id" },
        { "entry_face_id",    *entryId },
        { "start_x_mm",       in.start_x_mm },
        { "start_y_mm",       in.start_y_mm },
        { "direction_xy",     { in.direction_xy.X(), in.direction_xy.Y(), in.direction_xy.Z() } },
        { "length_mm",        in.length_mm },
        { "hook_height_mm",   in.hook_height_mm },
        { "hook_depth_mm",    in.hook_depth_mm },
        { "width_mm",         in.width_mm },
        { "wall_thickness_mm", in.wall_thickness_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "length_mm",            in.length_mm },
        { "width_mm",             in.width_mm },
        { "wall_thickness_mm",    in.wall_thickness_mm },
        { "hook_height_mm",       in.hook_height_mm },
        { "hook_depth_mm",        in.hook_depth_mm },
        { "extrude_dir",          { outward.X(), outward.Y(), outward.Z() } },
        { "beam_dir",             { xLoc.X(), xLoc.Y(), xLoc.Z() } },
        { "perpendicular_faces",  2 },     // beam top + hook back (L corner)
    };
    ToolingMeta tooling;
    tooling.tool_type        = "mold_feature";
    const double beamVol = in.length_mm * in.width_mm * in.wall_thickness_mm;
    const double hookVol = in.hook_depth_mm * in.width_mm * in.hook_height_mm;
    tooling.stock_removed_mm3 = -(beamVol + hookVol);
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::snap_fit applied: len={} hook(d={},h={}) wall={} faces {}→{}",
                  in.length_mm, in.hook_depth_mm, in.hook_height_mm,
                  in.wall_thickness_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay (high fidelity) + topology heuristic that looks for a
// L-shaped protrusion via face-count near a step edge.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "start_x_mm",       f.params.value("start_x_mm",       0.0) },
            { "start_y_mm",       f.params.value("start_y_mm",       0.0) },
            { "direction_xy",     f.params.value("direction_xy",     json::array({1.0,0.0,0.0})) },
            { "length_mm",        f.params.value("length_mm",        0.0) },
            { "hook_height_mm",   f.params.value("hook_height_mm",   0.0) },
            { "hook_depth_mm",    f.params.value("hook_depth_mm",    0.0) },
            { "width_mm",         f.params.value("width_mm",         0.0) },
            { "wall_thickness_mm", f.params.value("wall_thickness_mm", 0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology heuristic: look for a top face of the L that has a step.
    // For slice-1 we only flag "candidate" with low confidence; topology
    // recognition of an L is non-trivial after Boolean fuse simplification.
    const auto wpBb = pr::optimalBbox(wp.shape());
    int candidate = -1;
    double bestScore = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            const gp_Dir n = wp.faceNormal(i);
            if (std::abs(n.Z()) < 0.9) continue;            // top/bottom-like
        } catch (...) { continue; }
        const auto fb = pr::optimalBbox(wp.face(i));
        const double area = wp.faceArea(i);
        const double a = std::max(fb.dx(), fb.dy());
        const double b = std::min(fb.dx(), fb.dy());
        if (b < 1e-3) continue;
        if (a / b < 3.0) continue;
        if (area > wpBb.dx() * wpBb.dy() * 0.5) continue;     // too big
        // Score by aspect ratio.
        const double score = a / b;
        if (score > bestScore) { bestScore = score; candidate = i; }
    }
    if (candidate < 0) return out;

    const auto fb = pr::optimalBbox(wp.face(candidate));
    json rec = {
        { "start_x_mm",       fb.xMin },
        { "start_y_mm",       (fb.yMin + fb.yMax) / 2.0 },
        { "direction_xy",     json::array({ 1.0, 0.0, 0.0 }) },
        { "length_mm",        fb.dx() },
        { "hook_height_mm",   0.5 },
        { "hook_depth_mm",    0.3 },
        { "width_mm",         fb.dy() },
        { "wall_thickness_mm", fb.dz() },
    };
    json matched = { { "candidate_top_face_id", candidate } };
    out.push_back(RecognizedFeature{ kSkillId, rec, 0.4, matched });
    return out;
}

}  // namespace koocadcam::skill::snap_fit
