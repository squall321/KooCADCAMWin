// @lat: [[engine/skills#honing_op]]

#include "honing_op.hpp"

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

namespace koocadcam::skill::honing_op {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.removal_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "honing_op removal_mm must be > 0");
    }
    if (in.removal_mm > 0.0 && in.removal_mm < 0.0005) {
        r.add("DFM-HN-MIN", "error",
              "honing_op removal_mm " + std::to_string(in.removal_mm) +
              " mm < min 0.0005 mm — below hone-stone cut threshold");
    }
    if (in.removal_mm > 0.02) {
        r.add("DFM-HN-MAX", "error",
              "honing_op removal_mm " + std::to_string(in.removal_mm) +
              " mm > max 0.02 mm — exceeds hone single-pass limit "
              "(use internal_grind for larger removal)");
    }

    if (in.cross_hatch_angle_deg < 0.0 || in.cross_hatch_angle_deg > 90.0) {
        r.add("DFM-HN-ANGLE", "warning",
              "honing_op cross_hatch_angle_deg " +
              std::to_string(in.cross_hatch_angle_deg) +
              "° outside typical [0, 90] range");
    }

    auto cylId = wp.resolve(in.bore_datum);
    if (!cylId) {
        r.add("DFM-HN-DATUM", "error",
              "honing_op: bore_datum did not resolve to any face");
    } else if (!wp.isFaceCylinder(*cylId)) {
        r.add("DFM-HN-DATUM", "error",
              "honing_op: resolved face is not cylindrical (face_id=" +
              std::to_string(*cylId) + ")");
    } else {
        BRepAdaptor_Surface surf(wp.face(*cylId));
        const double radius = surf.Cylinder().Radius();
        const double targetDia = 2.0 * (radius + in.removal_mm);
        if (targetDia < 3.0) {
            r.add("DFM-HN-DIA", "error",
                  "honing_op: target bore diameter " +
                  std::to_string(targetDia) + " mm < 3 mm — hone holder will "
                  "not clear");
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
        std::string msg = "honing_op DFM failed:";
        for (const auto& f : dfm.findings) {
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        }
        throw SkillError(msg);
    }

    auto cylId = wp.resolve(in.bore_datum);
    if (!cylId)
        throw SkillError("honing_op: bore_datum unresolved");
    if (!wp.isFaceCylinder(*cylId))
        throw SkillError("honing_op: resolved face is not cylindrical");

    // 2) Recover axis + extent of the existing bore.
    const TopoDS_Face& cylFace = wp.face(*cylId);
    auto spanOpt = spanOfCylFace(cylFace);
    if (!spanOpt)
        throw SkillError("honing_op: cylindrical face has < 2 bounding "
                         "circular edges, cannot determine extent");
    const CylSpan& span = *spanOpt;

    const double oldRadius = span.radius;
    const double newRadius = oldRadius + in.removal_mm;
    const double extent    = span.centerHigh.Distance(span.centerLow);

    // 3) Build the (slightly larger) cylinder cutter.  When removal_mm is
    //    near OCCT's modelling tolerance the Boolean produces a near-no-op
    //    in shape but always succeeds; the signature still records the
    //    honing intent so process planning carries it forward.
    const gp_Dir adir = span.axis.Direction();
    const double kOverhang = 0.05;

    const gp_Pnt cutterBase(
        span.centerLow.X() - adir.X() * kOverhang,
        span.centerLow.Y() - adir.Y() * kOverhang,
        span.centerLow.Z() - adir.Z() * kOverhang);
    const gp_Ax2 cutterAx(cutterBase, adir);
    const TopoDS_Shape cutter = pr::cylinder(cutterAx, newRadius,
                                             extent + 2.0 * kOverhang);

    // 4) Cut.  For sub-tolerance removals OCCT may return a shape that's
    //    geometrically near-identical to the input — this is acceptable;
    //    the signature is the authoritative record of the operation.
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // 5) Signature
    json params = {
        { "bore_face_id",           *cylId },
        { "old_radius_mm",          oldRadius },
        { "new_radius_mm",          newRadius },
        { "removal_mm",             in.removal_mm },
        { "cross_hatch_angle_deg",  in.cross_hatch_angle_deg },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
        { "axis_location",          { span.axis.Location().X(),
                                      span.axis.Location().Y(),
                                      span.axis.Location().Z() } },
        { "extent_mm",              extent },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "cylindrical_face_count", 1 },
        { "diameter_mm",            2.0 * newRadius },
        { "removal_mm",             in.removal_mm },
        { "cross_hatch_angle_deg",  in.cross_hatch_angle_deg },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "honing_stone";
    tooling.tool_dia_mm       = 2.0 * (newRadius - 0.05);   // expanding hone
    tooling.tool_length_mm    = extent * 0.6 + 30.0;        // stones shorter than bore
    tooling.tool_material     = "silicon_carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 200.0;                      // very slow — reciprocating
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = M_PI * (newRadius * newRadius - oldRadius * oldRadius)
                              * extent;
    tooling.est_cycle_time_s  = std::max(15.0, extent / 5.0);  // hones are slow
    tooling.extra = json{
        { "cross_hatch_angle_deg", in.cross_hatch_angle_deg },
        { "abrasive_grit",         "400-grit" },
        { "coolant",               "honing_oil" },
        { "operation_type",        "internal_hone" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::honing_op applied: oldR={} +Δ{} → newR={} "
                  "xhatch={}° faces {}→{}",
                  oldRadius, in.removal_mm, newRadius,
                  in.cross_hatch_angle_deg,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A honed surface differs from a reamed / ground bore only in surface
// pattern (cross-hatch).  At our coarse modelling fidelity that pattern is
// invisible — recognize() returns empty.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    (void)wp;
    return {};
}

}  // namespace koocadcam::skill::honing_op
