// @lat: [[engine/skills#internal_grind]]

#include "internal_grind.hpp"

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

namespace koocadcam::skill::internal_grind {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.removal_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "internal_grind removal_mm must be > 0");
    }
    if (in.removal_mm > 0.0 && in.removal_mm < 0.005) {
        r.add("DFM-IG-MIN", "error",
              "internal_grind removal_mm " + std::to_string(in.removal_mm) +
              " mm < min 0.005 mm — below grinding-stone cut threshold");
    }
    if (in.removal_mm > 0.1) {
        r.add("DFM-IG-MAX", "error",
              "internal_grind removal_mm " + std::to_string(in.removal_mm) +
              " mm > max 0.1 mm — exceeds single-pass internal-grind limit");
    }

    auto cylId = wp.resolve(in.cylindrical_face_datum);
    if (!cylId) {
        r.add("DFM-IG-DATUM", "error",
              "internal_grind: cylindrical_face_datum did not resolve to any face");
    } else if (!wp.isFaceCylinder(*cylId)) {
        r.add("DFM-IG-DATUM", "error",
              "internal_grind: resolved face is not cylindrical (face_id=" +
              std::to_string(*cylId) + ")");
    } else {
        BRepAdaptor_Surface surf(wp.face(*cylId));
        const double radius = surf.Cylinder().Radius();
        const double targetDia = 2.0 * (radius + in.removal_mm);
        if (targetDia < 5.0) {
            r.add("DFM-IG-DIA", "error",
                  "internal_grind: target bore diameter " +
                  std::to_string(targetDia) + " mm < 5 mm — too small for a "
                  "grinding quill (use ream)");
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
        std::string msg = "internal_grind DFM failed:";
        for (const auto& f : dfm.findings) {
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        }
        throw SkillError(msg);
    }

    auto cylId = wp.resolve(in.cylindrical_face_datum);
    if (!cylId)
        throw SkillError("internal_grind: cylindrical_face_datum unresolved");
    if (!wp.isFaceCylinder(*cylId))
        throw SkillError("internal_grind: resolved face is not cylindrical");

    // 2) Recover axis + extent of the existing bore.
    const TopoDS_Face& cylFace = wp.face(*cylId);
    auto spanOpt = spanOfCylFace(cylFace);
    if (!spanOpt)
        throw SkillError("internal_grind: cylindrical face has < 2 bounding "
                         "circular edges, cannot determine extent");
    const CylSpan& span = *spanOpt;

    const double oldRadius = span.radius;
    const double newRadius = oldRadius + in.removal_mm;
    const double extent    = span.centerHigh.Distance(span.centerLow);

    // 3) Build cutter: cylinder at the new (larger) radius coaxial with the
    //    existing bore, slightly longer for clean Boolean intersection.
    const gp_Dir adir = span.axis.Direction();
    const double kOverhang = 0.05;

    const gp_Pnt cutterBase(
        span.centerLow.X() - adir.X() * kOverhang,
        span.centerLow.Y() - adir.Y() * kOverhang,
        span.centerLow.Z() - adir.Z() * kOverhang);
    const gp_Ax2 cutterAx(cutterBase, adir);
    const TopoDS_Shape cutter = pr::cylinder(cutterAx, newRadius,
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
    tooling.tool_type         = "grinding_quill";
    tooling.tool_dia_mm       = 2.0 * (newRadius - 0.5);   // stone slightly smaller than bore
    tooling.tool_length_mm    = extent * 1.5 + 20.0;
    tooling.tool_material     = "cubic_boron_nitride";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 4500.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = M_PI * (newRadius * newRadius - oldRadius * oldRadius)
                              * extent;
    tooling.est_cycle_time_s  = std::max(5.0, extent / 12.0);
    tooling.extra = json{
        { "surface_finish",  in.surface_finish },
        { "abrasive_grit",   "120-grit" },
        { "coolant",         "flood" },
        { "grind_type",      "internal_ID" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::internal_grind applied: oldR={} +Δ{} → newR={} "
                  "extent={} faces {}→{}",
                  oldRadius, in.removal_mm, newRadius, extent,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// An internally ground bore is topologically identical to a drilled or
// reamed bore — process metadata is required to differentiate.  We return
// empty so that drill_hole / ream / bore_cylindrical carry the canonical
// candidates; internal_grind is metadata-only.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    (void)wp;
    return {};
}

}  // namespace koocadcam::skill::internal_grind
