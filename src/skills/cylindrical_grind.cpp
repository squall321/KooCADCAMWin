// @lat: [[engine/skills#cylindrical_grind]]

#include "cylindrical_grind.hpp"

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
#include <optional>

namespace koocadcam::skill::cylindrical_grind {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.removal_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "cylindrical_grind removal_mm must be > 0");
    }
    if (in.removal_mm > 0.0 && in.removal_mm < 0.005) {
        r.add("DFM-CG-MIN", "error",
              "cylindrical_grind removal_mm " + std::to_string(in.removal_mm) +
              " mm < min 0.005 mm — below grinding-wheel cut threshold");
    }
    if (in.removal_mm > 0.1) {
        r.add("DFM-CG-MAX", "error",
              "cylindrical_grind removal_mm " + std::to_string(in.removal_mm) +
              " mm > max 0.1 mm — exceeds single-pass cylindrical-grind limit");
    }

    auto cylId = wp.resolve(in.cylindrical_face_datum);
    if (!cylId) {
        r.add("DFM-CG-DATUM", "error",
              "cylindrical_grind: cylindrical_face_datum did not resolve to any face");
    } else if (!wp.isFaceCylinder(*cylId)) {
        r.add("DFM-CG-DATUM", "error",
              "cylindrical_grind: resolved face is not cylindrical (face_id=" +
              std::to_string(*cylId) + ")");
    } else {
        BRepAdaptor_Surface surf(wp.face(*cylId));
        const double radius = surf.Cylinder().Radius();
        if (radius - in.removal_mm <= 0.0) {
            r.add("DFM-CG-RADIUS", "error",
                  "cylindrical_grind: removal " + std::to_string(in.removal_mm) +
                  " mm exceeds existing radius " + std::to_string(radius) + " mm");
        }
    }
    return r;
}

// ── Synthesis helpers ────────────────────────────────────────────────────

namespace {

struct CylSpan
{
    gp_Pnt centerHigh;
    gp_Pnt centerLow;
    double radius = 0.0;
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
        std::string msg = "cylindrical_grind DFM failed:";
        for (const auto& f : dfm.findings) {
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        }
        throw SkillError(msg);
    }

    auto cylId = wp.resolve(in.cylindrical_face_datum);
    if (!cylId)
        throw SkillError("cylindrical_grind: cylindrical_face_datum unresolved");
    if (!wp.isFaceCylinder(*cylId))
        throw SkillError("cylindrical_grind: resolved face is not cylindrical");

    // 2) Recover axis + radius + axial extent of the existing outer cylinder.
    const TopoDS_Face& cylFace = wp.face(*cylId);
    auto spanOpt = spanOfCylFace(cylFace);
    if (!spanOpt)
        throw SkillError("cylindrical_grind: cylindrical face has < 2 bounding "
                         "circular edges, cannot determine extent");
    const CylSpan& span = *spanOpt;

    const double oldRadius = span.radius;
    const double newRadius = oldRadius - in.removal_mm;
    const double extent    = span.centerHigh.Distance(span.centerLow);

    // 3) Build the cutter: an annular ring between newRadius and (oldRadius
    //    + ε) coaxial with the existing cylinder.  The +ε on the outer wall
    //    guarantees the Boolean cut cleanly removes the outer skin even in
    //    the presence of floating-point noise.
    const gp_Dir adir = span.axis.Direction();
    const double kOverhang  = 0.05;
    const double kOuterEpsilon = std::max(0.01, in.removal_mm * 0.5);

    const gp_Pnt cutterBase(
        span.centerLow.X() - adir.X() * kOverhang,
        span.centerLow.Y() - adir.Y() * kOverhang,
        span.centerLow.Z() - adir.Z() * kOverhang);
    const gp_Ax2 cutterAx(cutterBase, adir);

    const TopoDS_Shape cutter = pr::annularRing(
        cutterAx,
        oldRadius + kOuterEpsilon,
        newRadius,
        extent + 2.0 * kOverhang);

    // 4) Cut
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // 5) Build signature
    json params = {
        { "cylindrical_face_id", *cylId },
        { "old_radius_mm",       oldRadius },
        { "new_radius_mm",       newRadius },
        { "removal_mm",          in.removal_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "axis_location",       { span.axis.Location().X(),
                                   span.axis.Location().Y(),
                                   span.axis.Location().Z() } },
        { "extent_mm",           extent },
        { "surface_finish",      in.surface_finish },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "cylindrical_face_count", 1 },
        { "diameter_mm",            2.0 * newRadius },
        { "removal_mm",             in.removal_mm },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
        { "surface_finish",         in.surface_finish },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "grinding_wheel";
    tooling.tool_dia_mm       = 300.0;          // typical OD grinder wheel
    tooling.tool_length_mm    = 25.0;
    tooling.tool_material     = "aluminum_oxide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 6500.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Annulus volume = π × (Rold² − Rnew²) × extent
    tooling.stock_removed_mm3 = M_PI * (oldRadius * oldRadius - newRadius * newRadius)
                              * extent;
    tooling.est_cycle_time_s  = std::max(3.0, extent / 20.0);
    tooling.extra = json{
        { "surface_finish",  in.surface_finish },
        { "abrasive_grit",   "80-grit" },
        { "coolant",         "flood" },
        { "grind_type",      "external_OD" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cylindrical_grind applied: oldR={} −Δ{} → newR={} "
                  "extent={} faces {}→{}",
                  oldRadius, in.removal_mm, newRadius, extent,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// External-grind topology is identical to a plain cylindrical face — without
// process history we cannot infer "ground" status from shape alone.
// Heuristic: ground ODs tend to land on precision-tolerance values (radii
// ending in .500 / .250 / .000 mm).  We emit LOW-confidence candidates
// only when a cylindrical face's radius is close to such a value.

namespace {

// Is `r` within `tol` mm of a "round" value (multiples of 0.25)?
bool isRoundValue(double r, double tol = 0.01)
{
    const double scaled = r * 4.0;          // multiples of 0.25 ↔ integers
    const double nearest = std::round(scaled);
    return std::abs(scaled - nearest) <= tol * 4.0;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        auto spanOpt = spanOfCylFace(cylFace);
        if (!spanOpt) continue;
        const CylSpan& span = *spanOpt;

        const double radius = span.radius;
        const double extent = span.centerHigh.Distance(span.centerLow);
        if (extent < 1e-6) continue;

        // Only flag if radius lands on a "precision" value.
        if (!isRoundValue(radius)) continue;

        // External vs internal: external cylinders are "outies" — their face
        // normal points outward (away from axis).  We approximate by checking
        // adjacent face count / signed orientation is hard to do cleanly
        // here; accept all "round" cylinder faces as candidates and let the
        // recognizer pipeline disambiguate.

        json recovered = {
            { "cylindrical_face_id", fIdx },
            { "old_radius_mm",       radius + 0.02 },   // assume 0.02 mm removal
            { "new_radius_mm",       radius },
            { "removal_mm",          0.02 },
            { "axis_dir",            { span.axis.Direction().X(),
                                       span.axis.Direction().Y(),
                                       span.axis.Direction().Z() } },
            { "extent_mm",           extent },
            { "surface_finish",      "ra_0.4" },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "radius_mm",           radius },
            { "note",                "ground-OD candidate inferred from "
                                     "precision radius value; metadata required "
                                     "for confirmation" },
        };
        // Low confidence: identical topology to a turned cylinder.
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.30, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::cylindrical_grind
