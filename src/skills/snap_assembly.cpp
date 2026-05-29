// @lat: [[engine/skills#snap_assembly]]

#include "snap_assembly.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::snap_assembly {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // DFM-SA-WIDTH / DFM-SA-LENGTH — minimum mating-tab footprint.
    // Standards reference: Bayer Snap-Fit Design Manual (Bayer/Covestro
    // engineering guide) §3 "Cantilever snap-fits" specifies a minimum
    // moulded tab/cavity wall of ≈ 1 mm so the cavity walls can be filled
    // reliably in standard ABS/PC injection moulds.  SPI engineering
    // tolerance class B (commercial moulds) likewise treats 1 mm as the
    // minimum reliably-filled cavity dimension.
    constexpr double kMinSnapCavityFootprintMm = 1.0;  // Bayer Snap-Fit Design Manual §3 / SPI class B
    if (in.cavity_width_mm < kMinSnapCavityFootprintMm) {
        r.add("DFM-SA-WIDTH", "error",
              "snap_assembly cavity_width_mm " + std::to_string(in.cavity_width_mm) +
              " < 1.0 mm minimum (Bayer Snap-Fit Design Manual §3 / SPI class B)");
    }
    if (in.cavity_length_mm < kMinSnapCavityFootprintMm) {
        r.add("DFM-SA-LENGTH", "error",
              "snap_assembly cavity_length_mm " + std::to_string(in.cavity_length_mm) +
              " < 1.0 mm minimum (Bayer Snap-Fit Design Manual §3 / SPI class B)");
    }
    // DFM-SA-DEPTH — usable engagement depth for an injection-moulded snap.
    // Standards reference: Bayer Snap-Fit Design Manual §3.2 "Cantilever
    // dimensions" tabulates allowable deflection y_max ≈ 0.5–10 mm for
    // common engineering thermoplastics (ABS, PC, POM, PA66) at the
    // material's working strain.  Below 0.5 mm the snap cannot reliably
    // engage; above 10 mm the cantilever exceeds permissible strain and
    // typically yields.
    constexpr double kMinSnapDepthMm = 0.5;   // Bayer Snap-Fit Design Manual §3.2 min y_max
    constexpr double kMaxSnapDepthMm = 10.0;  // Bayer Snap-Fit Design Manual §3.2 max y_max for engineering plastics
    if (in.cavity_depth_mm < kMinSnapDepthMm || in.cavity_depth_mm > kMaxSnapDepthMm) {
        r.add("DFM-SA-DEPTH", "error",
              "snap_assembly cavity_depth_mm " + std::to_string(in.cavity_depth_mm) +
              " outside [0.5, 10.0] mm (Bayer Snap-Fit Design Manual §3.2 cantilever deflection range)");
    }
    if (in.tab_engage_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "snap_assembly tab_engage_mm must be > 0");
    } else if (in.cavity_depth_mm > 0.0 &&
               in.tab_engage_mm >= in.cavity_depth_mm) {
        r.add("DFM-SA-TAB", "error",
              "snap_assembly tab_engage_mm " + std::to_string(in.tab_engage_mm) +
              " >= cavity_depth_mm " + std::to_string(in.cavity_depth_mm) +
              " — tab would bottom out");
    }
    // insertion_axis_xy must be in XY plane (analogous to snap_fit).
    if (std::abs(in.insertion_axis_xy.Z()) > 1e-3) {
        r.add("DFM-INPUT", "error",
              "snap_assembly insertion_axis_xy must be in XY plane (Z component must be 0)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "snap_assembly DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("snap_assembly: entry_face datum unresolved");

    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("snap_assembly: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();

    // For slice-1, restrict to ±Z normals (consistent with snap_fit).
    if (std::abs(std::abs(outward.Z()) - 1.0) > 1e-2) {
        throw SkillError("snap_assembly: slice-1 supports only ±Z entry_face normals");
    }

    // Local frame:
    //   xLoc = insertion_axis_xy   (along cavity LENGTH)
    //   yLoc = outward × xLoc      (cavity WIDTH direction, in entry plane)
    //   inward = -outward          (DEPTH direction, into the workpiece)
    gp_Vec xVec(in.insertion_axis_xy);
    if (xVec.Magnitude() < 1e-9) {
        throw SkillError("snap_assembly: insertion_axis_xy has zero magnitude");
    }
    xVec.Normalize();
    gp_Vec yVec = gp_Vec(outward).Crossed(xVec);
    if (yVec.Magnitude() < 1e-9) {
        throw SkillError("snap_assembly: insertion_axis_xy parallel to entry-face normal");
    }
    yVec.Normalize();

    const gp_Dir xLoc(xVec.X(), xVec.Y(), xVec.Z());
    const gp_Dir yLoc(yVec.X(), yVec.Y(), yVec.Z());
    const gp_Dir inward(-outward.X(), -outward.Y(), -outward.Z());

    // Project the cavity centre onto the entry plane.
    auto projectOntoPlane = [&](double x, double y) {
        const gp_Pnt P0 = entryPlane.Location();
        const gp_Dir n  = entryPlane.Axis().Direction();
        if (std::abs(n.Z()) < 1e-9) return gp_Pnt(x, y, P0.Z());
        const double z = P0.Z() - (n.X() * (x - P0.X()) + n.Y() * (y - P0.Y())) / n.Z();
        return gp_Pnt(x, y, z);
    };
    const gp_Pnt centrePnt = projectOntoPlane(in.position_x_mm, in.position_y_mm);

    // Build the cutter box.
    //
    // gp_Ax2(P, V, Vx) interprets V as LOCAL Z and Vx as LOCAL X.  We pick
    //   V  = inward  → DZ extrudes into the workpiece by cavity_depth
    //   Vx = xLoc    → DX = cavity_length along xLoc
    // then DY = V × Vx = inward × xLoc = -(outward × xLoc) = -yLoc, so we
    // must position the box origin on the +yLoc side to keep the cavity
    // centred — i.e. origin = centre - (length/2) xLoc + (width/2) yLoc.
    //
    // Push the cutter slightly ABOVE the entry face (along outward) by
    // kEmbed so the cut is clean.
    constexpr double kEmbed = 0.02;

    const gp_Pnt cutterOrigin(
        centrePnt.X() - (in.cavity_length_mm / 2.0) * xLoc.X()
                      + (in.cavity_width_mm  / 2.0) * yLoc.X()
                      + kEmbed * outward.X(),
        centrePnt.Y() - (in.cavity_length_mm / 2.0) * xLoc.Y()
                      + (in.cavity_width_mm  / 2.0) * yLoc.Y()
                      + kEmbed * outward.Y(),
        centrePnt.Z() - (in.cavity_length_mm / 2.0) * xLoc.Z()
                      + (in.cavity_width_mm  / 2.0) * yLoc.Z()
                      + kEmbed * outward.Z());

    const gp_Ax2 cutterAx(cutterOrigin, inward, xLoc);
    const TopoDS_Shape cutter = pr::box(cutterAx,
                                        in.cavity_length_mm,
                                        in.cavity_width_mm,
                                        in.cavity_depth_mm + kEmbed);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Signature
    json params = {
        { "entry_face_id",      *entryId },
        { "position_x_mm",      in.position_x_mm },
        { "position_y_mm",      in.position_y_mm },
        { "insertion_axis_xy",  { in.insertion_axis_xy.X(),
                                  in.insertion_axis_xy.Y(),
                                  in.insertion_axis_xy.Z() } },
        { "cavity_length_mm",   in.cavity_length_mm },
        { "cavity_width_mm",    in.cavity_width_mm },
        { "cavity_depth_mm",    in.cavity_depth_mm },
        { "tab_engage_mm",      in.tab_engage_mm },
        { "partner_part",       in.partner_part },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "planar_wall_count",          4 },
        { "bottom_planar_face_present", true },
        { "cavity_length_mm",           in.cavity_length_mm },
        { "cavity_width_mm",            in.cavity_width_mm },
        { "cavity_depth_mm",            in.cavity_depth_mm },
        { "outward_normal",             { outward.X(), outward.Y(), outward.Z() } },
        { "insertion_axis",             { xLoc.X(), xLoc.Y(), xLoc.Z() } },
        { "additive",                   false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    // Tool diameter sized to fit corners — half the smaller cavity dimension.
    tooling.tool_dia_mm       = std::min(in.cavity_length_mm, in.cavity_width_mm) * 0.5;
    tooling.tool_length_mm    = in.cavity_depth_mm * 2.0 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double cavityVol = in.cavity_length_mm * in.cavity_width_mm * in.cavity_depth_mm;
    tooling.stock_removed_mm3 = cavityVol;
    tooling.est_cycle_time_s  = std::max(1.0, cavityVol / 200.0);
    tooling.extra = {
        { "fastener_type",      "snap_tab" },
        { "tab_engage_mm",      in.tab_engage_mm },
        { "partner_part",       in.partner_part },
        { "assembly_step",      "insert_tab_along_insertion_axis_until_engage_depth" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::snap_assembly applied: cavity {}×{}×{} faces {}→{}",
                  in.cavity_length_mm, in.cavity_width_mm, in.cavity_depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay (confidence 1.0).  Geometric fallback looks for small
// rectangular pockets: 4 axis-aligned planar walls + 1 bottom planar face,
// footprint area < 50 mm².  Confidence 0.5 — indistinguishable from a plain
// mill_rect_pocket without metadata.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // 1) History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",     f.params.value("position_x_mm",     0.0) },
            { "position_y_mm",     f.params.value("position_y_mm",     0.0) },
            { "insertion_axis_xy", f.params.value("insertion_axis_xy",
                                                  json::array({1.0,0.0,0.0})) },
            { "cavity_length_mm",  f.params.value("cavity_length_mm",  0.0) },
            { "cavity_width_mm",   f.params.value("cavity_width_mm",   0.0) },
            { "cavity_depth_mm",   f.params.value("cavity_depth_mm",   0.0) },
            { "tab_engage_mm",     f.params.value("tab_engage_mm",     0.0) },
            { "partner_part",      f.params.value("partner_part",      std::string()) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // 2) Geometric fallback — look for a candidate horizontal bottom face
    //    whose bbox area is in the snap-cavity range.  We do NOT verify the
    //    walls strictly; confidence is intentionally low (0.5) because a
    //    mill_rect_pocket of the same size is indistinguishable.
    const auto wpBb = pr::optimalBbox(wp.shape());
    const double topZ = wpBb.zMax;

    int   bestFace = -1;
    double bestArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); }
        catch (...) { continue; }
        // Bottom faces of a top-entry cavity point UPWARD (+Z) and sit BELOW
        // the workpiece top.
        if (std::abs(n.Z() - 1.0) > 1e-2) continue;

        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() >= topZ - 1e-3) continue;   // not a cavity bottom (== top face)

        const auto fb = pr::optimalBbox(wp.face(i));
        const double fa = fb.dx() * fb.dy();
        if (fa < 1.0 || fa > 50.0) continue;
        // Pick the largest qualifying — most likely the snap_assembly cavity.
        if (fa > bestArea) { bestArea = fa; bestFace = i; }
    }
    if (bestFace < 0) return out;

    const auto fb = pr::optimalBbox(wp.face(bestFace));
    const gp_Pnt c = wp.faceCenter(bestFace);
    const double depth = topZ - c.Z();
    if (depth < 0.5 || depth > 10.0) return out;

    // Length = longer side, width = shorter side (heuristic).
    const double len = std::max(fb.dx(), fb.dy());
    const double wid = std::min(fb.dx(), fb.dy());

    // Assume insertion axis along +X if length is along X, else +Y.
    json axis = (fb.dx() >= fb.dy())
        ? json::array({ 1.0, 0.0, 0.0 })
        : json::array({ 0.0, 1.0, 0.0 });

    json rec = {
        { "position_x_mm",     c.X() },
        { "position_y_mm",     c.Y() },
        { "insertion_axis_xy", axis },
        { "cavity_length_mm",  len },
        { "cavity_width_mm",   wid },
        { "cavity_depth_mm",   depth },
        { "tab_engage_mm",     depth * 0.5 },   // unknown — guess half-depth
        { "partner_part",      std::string() },
    };
    json matched = {
        { "bottom_face_id", bestFace },
        { "footprint_mm2",  bestArea },
    };
    out.push_back(RecognizedFeature{ kSkillId, rec, 0.5, matched });
    return out;
}

}  // namespace koocadcam::skill::snap_assembly
