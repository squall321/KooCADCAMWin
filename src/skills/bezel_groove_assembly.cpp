// @lat: [[engine/skills#bezel_groove_assembly]]

#include "bezel_groove_assembly.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <BRepAdaptor_Surface.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::bezel_groove_assembly {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// DIN 471-style snap-ring retention groove gates.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.outer_dia_mm <= 0.0 || in.inner_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bezel_groove_assembly: outer_dia and inner_dia must be > 0");
        return r;
    }
    if (in.inner_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-INPUT", "error",
              "bezel_groove_assembly: inner_dia (" +
              std::to_string(in.inner_dia_mm) + ") must be < outer_dia (" +
              std::to_string(in.outer_dia_mm) + ")");
        return r;
    }

    const double width = (in.outer_dia_mm - in.inner_dia_mm) / 2.0;
    if (width < 0.4) {
        r.add("DFM-BEZEL-WIDTH", "error",
              "bezel_groove_assembly: groove width " + std::to_string(width) +
              " mm < min 0.4 mm (DIN 471 snap-ring retention)");
    }

    if (in.groove_depth_mm < 0.3) {
        r.add("DFM-BEZEL-DEPTH", "error",
              "bezel_groove_assembly: groove_depth " +
              std::to_string(in.groove_depth_mm) +
              " mm < min 0.3 mm (insufficient snap-ring engagement)");
    }
    if (in.groove_depth_mm > in.outer_dia_mm / 8.0) {
        r.add("DFM-BEZEL-DEPTH", "error",
              "bezel_groove_assembly: groove_depth " +
              std::to_string(in.groove_depth_mm) +
              " mm > outer_dia/8 (" + std::to_string(in.outer_dia_mm / 8.0) +
              ") — over-deep cut, weakens retention land");
    }

    if (in.taper_deg < 15.0 || in.taper_deg > 60.0) {
        r.add("DFM-BEZEL-TAPER", "warning",
              "bezel_groove_assembly: taper_deg " +
              std::to_string(in.taper_deg) +
              " outside DIN 6784 watch bezel range [15, 60]");
    }

    if (in.bottom_fillet_mm < 0.05) {
        r.add("DFM-004", "warning",
              "bezel_groove_assembly: bottom_fillet " +
              std::to_string(in.bottom_fillet_mm) +
              " mm < min R 0.05 mm — stress concentrator");
    }

    // Case-thickness margin: groove sits within the case top.  If the
    // workpiece is too thin, the cut consumes the retention land.
    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double thickness = zMax - zMin;
        if (thickness < in.outer_dia_mm / 4.0) {
            r.add("DFM-BEZEL-WALL", "warning",
                  "bezel_groove_assembly: case thickness " +
                  std::to_string(thickness) + " mm < outer_dia/4 (" +
                  std::to_string(in.outer_dia_mm / 4.0) +
                  ") — retention land may be too thin");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain (4 ops):
//   1. outer annulus cylinder  (radius = outer_dia/2, height = groove_depth)
//   2. inner annulus cylinder  (radius = inner_dia/2, height = groove_depth)
//      → outer + inner FUSED then single cut (overlapping concentric cyls)
//      → produces the U-channel
//   3. tip chamfer cone        (frustum at entry edge — taper_deg)
//   4. bottom fillet relief    (thin annular ring at floor — relieves stress)
//
// We produce a single Workpiece (one final cut() call against the fused
// cutter compound) and a single FeatureSignature describing the whole bezel.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bezel_groove_assembly DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double rOuter = in.outer_dia_mm / 2.0;
    const double rInner = in.inner_dia_mm / 2.0;
    const double depth  = in.groove_depth_mm;

    // Resolve entry-face top Z (case top).  For slice-1 we use bbox top when
    // axis points -Z (standard).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    constexpr double kOverhang = 0.05;

    // Entry point for cutters on the case-top (z = topZ, cutter axis points -Z).
    const gp_Pnt entryPt(in.center_x_mm, in.center_y_mm, topZ + kOverhang);
    const gp_Ax2 ax(entryPt, gp_Dir(0.0, 0.0, -1.0));

    // Sub-feature 1: outer cylinder (annulus outer wall)
    const TopoDS_Shape outerCyl = pr::cylinder(ax, rOuter, depth + kOverhang);

    // Sub-feature 2: inner cylinder (annulus inner wall — to subtract from outer)
    // We don't cut directly with both: the groove is the ANNULUS between them.
    // Build the annular cutter via pr::annularRing.
    const TopoDS_Shape annularCutter = pr::annularRing(ax, rOuter, rInner, depth + kOverhang);

    // Sub-feature 3: tip chamfer cone — small frustum band on the entry edge.
    // Outer radius at top = rOuter; inner radius at top = rOuter − chamfer_w;
    // we subtract a small cone band of half-angle `taper_deg`.
    const double chamferRise = depth * 0.15;   // 15% of depth as chamfer rise
    const double chamferRun  = chamferRise * std::tan(in.taper_deg * M_PI / 180.0);
    // Cone tool spans from rOuter+chamferRun (just outside) at top to rOuter
    // at depth chamferRise (just below) — it CARVES the lip.
    const gp_Pnt coneOrigin(in.center_x_mm, in.center_y_mm, topZ + kOverhang);
    const gp_Ax2 coneAx(coneOrigin, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape chamferCone =
        pr::coneFrustum(coneAx, rOuter + chamferRun + 0.02,
                        rInner - chamferRun - 0.02 > 0.05
                            ? rInner - chamferRun - 0.02 : 0.05,
                        chamferRise + kOverhang);

    // Sub-feature 4: bottom fillet relief — tiny annular cut at the floor.
    // Approximated by a thin annular ring at depth − bottom_fillet (extra
    // depth equal to fillet radius) with radius range (rInner − f, rOuter + f)
    // clipped to be only the FILLET BAND.  We approximate as a tiny annulus
    // at the corner.
    const double f = in.bottom_fillet_mm;
    const TopoDS_Shape filletRelief = pr::annularRing(
        gp_Ax2(gp_Pnt(in.center_x_mm, in.center_y_mm, topZ - depth + kOverhang),
               gp_Dir(0.0, 0.0, -1.0)),
        rOuter + f, rInner - f > 0.05 ? rInner - f : 0.05,
        f + kOverhang);

    // Fuse all 4 sub-feature tools into one compound cutter — this is the
    // "fuse_first_then_cut" pattern from counterbore.cpp (overlapping ops).
    const TopoDS_Shape fused1 = pr::fuse(annularCutter, chamferCone);
    const TopoDS_Shape fused2 = pr::fuse(fused1, filletRelief);
    (void)outerCyl;   // outerCyl already represented inside annularCutter

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused2);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",        in.center_x_mm },
        { "center_y_mm",        in.center_y_mm },
        { "axis_dir",           { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "outer_dia_mm",       in.outer_dia_mm },
        { "inner_dia_mm",       in.inner_dia_mm },
        { "groove_depth_mm",    in.groove_depth_mm },
        { "taper_deg",          in.taper_deg },
        { "bottom_fillet_mm",   in.bottom_fillet_mm },
    };

    const double grooveWidth = (in.outer_dia_mm - in.inner_dia_mm) / 2.0;
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           4 },
        { "subfeatures", json::array({
            { { "op", "outer_annulus_cut" },  { "radius_mm", rOuter }, { "depth_mm", depth } },
            { { "op", "inner_annulus_cut" },  { "radius_mm", rInner }, { "depth_mm", depth } },
            { { "op", "tip_chamfer_cone" },   { "taper_deg", in.taper_deg }, { "rise_mm", chamferRise } },
            { { "op", "bottom_fillet" },      { "fillet_mm", in.bottom_fillet_mm } },
        }) },
        { "outer_dia_mm",               in.outer_dia_mm },
        { "inner_dia_mm",               in.inner_dia_mm },
        { "groove_depth_mm",            in.groove_depth_mm },
        { "groove_width_mm",            grooveWidth },
        { "taper_deg",                  in.taper_deg },
        { "bottom_fillet_mm",           in.bottom_fillet_mm },
        { "annular_face_count",         2 },          // outer + inner walls
        { "bottom_planar_face_present", true },
        { "axis_dir",                   { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "groove_tool;chamfer_tool;fillet_tool";
    tooling.tool_dia_mm       = grooveWidth;
    tooling.tool_length_mm    = in.groove_depth_mm * 1.5 + 3.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    // Volume estimate: annular volume + small chamfer/fillet correction.
    const double annularVol =
        M_PI * (rOuter * rOuter - rInner * rInner) * depth;
    tooling.stock_removed_mm3 = annularVol;
    tooling.est_cycle_time_s  = std::max(2.0, depth / 30.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "groove_tool" },  { "width_mm", grooveWidth } },
            { { "tool_type", "chamfer_tool" }, { "taper_deg", in.taper_deg } },
            { { "tool_type", "fillet_tool" },  { "radius_mm", in.bottom_fillet_mm } },
        } },
        { "feature_standard", "DIN 471 snap-ring retention groove" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::bezel_groove_assembly applied: outer={} inner={} depth={} taper={}",
                  in.outer_dia_mm, in.inner_dia_mm, in.groove_depth_mm, in.taper_deg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// PRIMARY (geometric): a bezel groove leaves a distinctive "two coaxial
// inward-facing cylindrical faces of differing radii sharing the same
// Z axis" pattern (outer wall + inner wall of the annular U-channel), plus
// at least one horizontal planar face at the groove floor.  We walk the
// faces, cluster cylindrical faces by axis location, and look for the
// outer/inner pair on a common axis.
//
// FALLBACK (metadata): replay from wp.features() at confidence 1.0.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // (a) Metadata replay — highest confidence.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source", "metadata_replay" },
            { "is_compound", true },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // (b) Geometric heuristic — look for coaxial Z-axis cylindrical pairs.
    if (wp.shape().IsNull()) return out;

    struct CylRec {
        int    faceIdx = -1;
        double radius  = 0.0;
        gp_Pnt loc;
        gp_Dir dir;
        double topZ    = 0.0;
        double botZ    = 0.0;
    };
    std::vector<CylRec> zCyls;
    zCyls.reserve(static_cast<size_t>(wp.faceCount()));
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder cy = s.Cylinder();
        const gp_Dir d = cy.Axis().Direction();
        if (std::abs(std::abs(d.Z()) - 1.0) > 1e-3) continue;
        CylRec rec;
        rec.faceIdx = i;
        rec.radius  = cy.Radius();
        rec.loc     = cy.Axis().Location();
        rec.dir     = d;
        zCyls.push_back(rec);
    }
    if (zCyls.size() < 2) return out;

    // Cluster by (X, Y) of axis location — pairs on the same axis form a
    // coaxial group.  bezel needs outer + inner cylinder pair where the
    // groove width = (rOuter - rInner) is small relative to rOuter.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    for (size_t a = 0; a < zCyls.size(); ++a) {
        for (size_t b = a + 1; b < zCyls.size(); ++b) {
            const auto& A = zCyls[a];
            const auto& B = zCyls[b];
            // Same axis (≤ 0.5 mm offset).
            const double dx = A.loc.X() - B.loc.X();
            const double dy = A.loc.Y() - B.loc.Y();
            if (std::hypot(dx, dy) > 0.5) continue;
            // Differ in radius.
            const double rO = std::max(A.radius, B.radius);
            const double rI = std::min(A.radius, B.radius);
            if (rO - rI < 0.4) continue;            // DFM-BEZEL-WIDTH floor
            if (rO - rI > rO * 0.5) continue;       // groove is annular, not a bore
            // outer radius must be reasonable (≥ 2 mm — watch-bezel scale).
            if (rO < 2.0) continue;

            json recovered = {
                { "center_x_mm",      A.loc.X() },
                { "center_y_mm",      A.loc.Y() },
                { "axis_dir",         { 0.0, 0.0, -1.0 } },
                { "outer_dia_mm",     2.0 * rO },
                { "inner_dia_mm",     2.0 * rI },
                { "groove_depth_mm",  std::max(0.3, (zMax - zMin) * 0.1) },
                { "taper_deg",        30.0 },
                { "bottom_fillet_mm", 0.15 },
            };
            json matched = {
                { "source",         "geometric_primary" },
                { "outer_face_id",  (A.radius >= B.radius) ? A.faceIdx : B.faceIdx },
                { "inner_face_id",  (A.radius <  B.radius) ? A.faceIdx : B.faceIdx },
                { "outer_radius_mm", rO },
                { "inner_radius_mm", rI },
                { "groove_width_mm", rO - rI },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, /*confidence*/ 0.70, matched });
            return out;   // first matching coaxial pair is the bezel
        }
    }
    return out;
}

}  // namespace koocadcam::skill::bezel_groove_assembly
