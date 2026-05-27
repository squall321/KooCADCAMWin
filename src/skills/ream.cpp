// @lat: [[engine/skills#ream]]

#include "ream.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::ream {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.enlarge_by_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "ream enlarge_by must be > 0");
    }
    if (in.enlarge_by_mm > 0.0 && in.enlarge_by_mm < 0.02) {
        r.add("DFM-REAM-MIN", "error",
              "ream enlarge_by " + std::to_string(in.enlarge_by_mm) +
              " mm < min 0.02 mm — below reamer-cut threshold");
    }
    if (in.enlarge_by_mm > 0.30) {
        r.add("DFM-REAM-MAX", "error",
              "ream enlarge_by " + std::to_string(in.enlarge_by_mm) +
              " mm > max 0.30 mm — single-pass ream limit (use boring or " +
              "second drill operation for larger removal)");
    }

    // Verify the datum resolves to a cylindrical face.
    auto cylId = wp.resolve(in.existing_hole_datum);
    if (!cylId) {
        r.add("DFM-REAM-DATUM", "error",
              "ream: existing_hole_datum did not resolve to any face");
    } else if (!wp.isFaceCylinder(*cylId)) {
        r.add("DFM-REAM-DATUM", "error",
              "ream: resolved face is not cylindrical (face_id=" +
              std::to_string(*cylId) + ")");
    }
    return r;
}

// ── Synthesis helpers ────────────────────────────────────────────────────

namespace {

// Find the two extreme circle-edge centers of a cylindrical face along its
// axis.  Returns (highEnd, lowEnd) along the axis direction.  If less than
// two circles are found, returns std::nullopt.
struct CylSpan
{
    gp_Pnt centerHigh;   // larger projection-along-axis
    gp_Pnt centerLow;    // smaller projection-along-axis
    double radius;
    gp_Ax1 axis;
};

std::optional<CylSpan> spanOfCylFace(const TopoDS_Face& cylFace)
{
    BRepAdaptor_Surface surf(cylFace);
    if (surf.GetType() != GeomAbs_Cylinder) return std::nullopt;
    const gp_Cylinder cyl = surf.Cylinder();
    const gp_Ax1 axis     = cyl.Axis();
    const double radius   = cyl.Radius();

    std::vector<gp_Pnt> centers;
    for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
        BRepAdaptor_Curve crv(e);
        if (crv.GetType() != GeomAbs_Circle) continue;
        const gp_Circ c = crv.Circle();
        if (std::abs(std::abs(c.Axis().Direction().Dot(axis.Direction())) - 1.0) > 1e-3)
            continue;
        if (std::abs(c.Radius() - radius) > 1e-3) continue;
        centers.push_back(c.Location());
    }
    if (centers.size() < 2) return std::nullopt;

    const gp_Dir adir = axis.Direction();
    auto proj = [&](const gp_Pnt& p) {
        return (p.X() - axis.Location().X()) * adir.X() +
               (p.Y() - axis.Location().Y()) * adir.Y() +
               (p.Z() - axis.Location().Z()) * adir.Z();
    };
    auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) { return proj(a) < proj(b); };
    const auto minIt = std::min_element(centers.begin(), centers.end(), cmp);
    const auto maxIt = std::max_element(centers.begin(), centers.end(), cmp);

    CylSpan span;
    span.centerLow  = *minIt;
    span.centerHigh = *maxIt;
    span.radius     = radius;
    span.axis       = axis;
    return span;
}

}  // namespace

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ream DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto cylId = wp.resolve(in.existing_hole_datum);
    if (!cylId)
        throw SkillError("ream: existing_hole_datum unresolved");
    if (!wp.isFaceCylinder(*cylId))
        throw SkillError("ream: resolved face is not cylindrical");

    // 2) Extract the existing bore's axis + radius + extent.
    const TopoDS_Face& cylFace = wp.face(*cylId);
    auto spanOpt = spanOfCylFace(cylFace);
    if (!spanOpt)
        throw SkillError("ream: cylindrical face has < 2 bounding circular edges, "
                         "cannot determine bore extent");
    const CylSpan& span = *spanOpt;

    const double oldRadius = span.radius;
    const double newRadius = oldRadius + in.enlarge_by_mm;
    const double extent    = span.centerHigh.Distance(span.centerLow);

    // 3) Build the cutter: a cylinder COAXIAL with the existing bore, at the
    //    new radius, slightly longer than the existing extent (to ensure the
    //    Boolean cleanly captures both ends).
    const gp_Dir adir = span.axis.Direction();
    const double kOverhang = 0.05;

    // Anchor the new cutter at the LOW end with a small upward overhang
    // (along -axis_dir from low).  Then extend by extent + 2·overhang.
    const gp_Pnt cutterBase(
        span.centerLow.X() - adir.X() * kOverhang,
        span.centerLow.Y() - adir.Y() * kOverhang,
        span.centerLow.Z() - adir.Z() * kOverhang);
    const gp_Ax2 cutterAx(cutterBase, adir);
    const TopoDS_Shape cutter = pr::cylinder(cutterAx, newRadius, extent + 2.0 * kOverhang);

    // 4) Cut
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // 5) Build signature.  Recover the entry XY from the existing bore so
    //    downstream tools (CAM) can recompute coolant / feed-direction.
    json params = {
        { "existing_hole_face_id", *cylId },
        { "old_radius_mm",         oldRadius },
        { "new_radius_mm",         newRadius },
        { "enlarge_by_mm",         in.enlarge_by_mm },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
        { "axis_location",         { span.axis.Location().X(),
                                     span.axis.Location().Y(),
                                     span.axis.Location().Z() } },
        { "extent_mm",             extent },
    };
    // The PATTERN.kind tag is what distinguishes a ream from a drill_hole in
    // the feature history.  Topologically the shape is the same as a wider
    // drill, but the process plan needs the ream metadata for CAM.
    json pattern = {
        { "kind",                   kSkillId },
        { "cylindrical_face_count", 1 },
        { "circular_edge_count",    2 },
        { "diameter_mm",            2.0 * newRadius },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
        { "from_drill_hole",        true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "reamer";
    tooling.tool_dia_mm       = 2.0 * newRadius;
    tooling.tool_length_mm    = extent * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 6;                  // multi-flute typical for reamers
    tooling.cutting_speed_sfm = 80.0;               // slow vs drill
    tooling.feed_per_tooth_mm = 0.04;
    // Stock removed is the annulus volume: π (rNew² − rOld²) × extent
    tooling.stock_removed_mm3 = M_PI * (newRadius * newRadius - oldRadius * oldRadius)
                              * extent;
    tooling.est_cycle_time_s  = std::max(1.0, extent / 25.0);  // slower than drill
    tooling.extra = {
        { "tolerance_grade", "H7" },
        { "surface_finish_Ra", 0.8 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ream applied: oldR={} +Δ{} → newR={}, extent={} faces {}→{}",
                  oldRadius, in.enlarge_by_mm, newRadius, extent,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// HARD alone — a reamed hole looks topologically identical to a slightly
// wider drill_hole.  We mark cylindrical bores whose diameter falls in a
// typical "reamed" range (e.g. 0.05 mm above a standard drill size) as
// LOW-confidence ream candidates so the process planner can flag them; the
// caller should typically prefer drill_hole's higher-confidence candidate.
//
// Strategy:
//   - For every cylindrical face that bounds 2 circles (a true bore), emit
//     a ream candidate at confidence 0.30 carrying the recovered diameter
//     and a guess at the "original" drill diameter (= recovered minus
//     0.10 mm radial, a common ream allowance).
//   - This is intentionally a HEURISTIC.  In production the upstream
//     feature-history JSON tells us the operation was a ream; recognize()
//     here is the fallback when metadata is lost.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        auto spanOpt = spanOfCylFace(cylFace);
        if (!spanOpt) continue;
        const CylSpan& span = *spanOpt;

        const double dia = 2.0 * span.radius;
        const double extent = span.centerHigh.Distance(span.centerLow);
        if (extent < 1e-6) continue;

        // Heuristic: assume the ream removed 0.10 mm (radial) on top of a
        // drilled pilot.  Caller can override if metadata exists.
        const double assumedEnlarge = 0.10;

        gp_Vec dirVec(span.centerHigh, span.centerLow);
        if (dirVec.Magnitude() < 1e-9) continue;
        dirVec.Normalize();

        // Confidence is intentionally low — ream looks like drill_hole.
        const double conf = 0.35;

        json recovered = {
            { "existing_hole_face_id", fIdx },
            { "old_radius_mm",         span.radius - assumedEnlarge },
            { "new_radius_mm",         span.radius },
            { "enlarge_by_mm",         assumedEnlarge },
            { "axis_dir",              { dirVec.X(), dirVec.Y(), dirVec.Z() } },
            { "axis_location",         { span.axis.Location().X(),
                                         span.axis.Location().Y(),
                                         span.axis.Location().Z() } },
            { "extent_mm",             extent },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "diameter_mm",         dia },
            { "note",                "ream looks identical to drill_hole; "
                                     "prefer drill_hole candidate unless "
                                     "process metadata says otherwise" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::ream
