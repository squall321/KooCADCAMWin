// @lat: [[engine/skills#spot_drill]]

#include "spot_drill.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cone.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::spot_drill {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Derived geometry ─────────────────────────────────────────────────────

double computeDepth(const Input& in)
{
    if (in.diameter_mm <= 0.0) return 0.0;
    if (in.cone_angle_deg <= 0.0 || in.cone_angle_deg >= 180.0) return 0.0;
    const double halfRad = (in.cone_angle_deg * 0.5) * M_PI / 180.0;
    const double t = std::tan(halfRad);
    if (t < 1e-9) return 0.0;
    return (in.diameter_mm * 0.5) / t;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "spot_drill diameter must be > 0");
    }
    if (in.diameter_mm > 0.0 && in.diameter_mm < 0.8) {
        r.add("DFM-002", "error",
              "spot_drill diameter " + std::to_string(in.diameter_mm) +
              " mm < min 0.8 mm");
    }
    if (in.cone_angle_deg < 60.0 || in.cone_angle_deg > 140.0) {
        r.add("DFM-SPOT-ANGLE", "error",
              "spot_drill cone angle " + std::to_string(in.cone_angle_deg) +
              " deg outside standard range [60, 140] (typical 90° / 120°)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spot_drill DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Resolve entry face datum
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("spot_drill: entry_face datum unresolved");

    const double depth = computeDepth(in);
    if (depth <= 1e-6)
        throw SkillError("spot_drill: degenerate cone depth (angle/dia mismatch)");

    // 3) Bbox-driven entry point (same convention as drill_hole / countersink)
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const gp_Dir adir = in.axis_dir;

    // Entry-plane point: where the cone top lives.  For the common ±Z case
    // we project onto zMax or zMin directly; otherwise fall back to a
    // bbox-margin point along -axis_dir.
    gp_Pnt entryPlanePoint(
        in.position_x_mm - adir.X() * bboxDiag,
        in.position_y_mm - adir.Y() * bboxDiag,
        (zMin + zMax) / 2.0 - adir.Z() * bboxDiag);
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        const double entryZ = (adir.Z() < 0) ? zMax : zMin;
        entryPlanePoint = gp_Pnt(in.position_x_mm, in.position_y_mm, entryZ);
    }

    // 4) Build the cone-frustum cutter.
    //    coneFrustum(axis, r1_at_origin, r2_at_+height, height):
    //      r1 = diameter/2  at entry plane
    //      r2 ≈ 0 (small non-zero to avoid OCCT degenerate apex)
    //      height = depth (cone extends inward along adir)
    //
    // Note (OCCT pitfall): r2 = 0 occasionally trips BRepPrimAPI_MakeCone's
    // degeneracy guard depending on solver version.  We pass a tiny positive
    // value (1e-3 mm) so the result is a true frustum with a vanishingly
    // small bottom disc — geometrically indistinguishable from a sharp apex
    // for any downstream check and safe across OCCT 8.0.
    const double r1 = in.diameter_mm * 0.5;
    const double r2 = 1e-3;
    const gp_Ax2 coneAx(entryPlanePoint, adir);
    const TopoDS_Shape cutter = pr::coneFrustum(coneAx, r1, r2, depth);

    // 5) Cut
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // 6) Build signature
    json params = {
        { "entry_face_kind",  "resolved_id" },
        { "entry_face_id",    *entryId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",      in.diameter_mm },
        { "cone_angle_deg",   in.cone_angle_deg },
        { "depth_mm",         depth },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "conical_face_count",         1 },
        { "cylindrical_face_count",     0 },
        { "circular_edge_count",        1 },
        { "bottom_planar_face_present", false },
        { "diameter_mm",                in.diameter_mm },
        { "cone_angle_deg",             in.cone_angle_deg },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "spot_drill";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = depth * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;       // conservative — spot drills run slower
    tooling.feed_per_tooth_mm = 0.025;
    // Cone volume = (1/3) π r² h
    tooling.stock_removed_mm3 = (M_PI / 3.0) * r1 * r1 * depth;
    tooling.est_cycle_time_s  = std::max(0.5, depth / 30.0);
    tooling.extra = {
        { "purpose", "centering_mark_before_drill_hole" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::spot_drill applied: dia={} angle={}° depth={} faces {}→{}",
                  in.diameter_mm, in.cone_angle_deg, depth,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern matching strategy:
//   1. Iterate every conical face.
//   2. The spot mark has ONLY a cone — no coaxial cylinder.  A countersink
//      would put a cylinder past the cone's small end; we therefore skip
//      cone faces whose small-end circle has any adjacent cylindrical face.
//   3. The cone should be "shallow": its larger circle radius is comparable
//      to its depth (depth ≈ r / tan(angle/2)) and the small end degenerates
//      to a vanishingly small disc (apex).
//   4. Recover:
//        diameter   = 2 × cone.rLarge
//        cone_angle = 2 × cone.SemiAngle (deg)
//        position   = large-circle center XY
//        depth      = computeDepth from recovered (diameter, angle)
//        axis_dir   = from largeCenter → smallCenter

namespace {

struct ConeInfo
{
    int     faceIdx       = -1;
    double  semiAngleRad  = 0.0;
    gp_Ax1  axis;
    double  rLarge        = 0.0;
    double  rSmall        = 0.0;
    gp_Pnt  largeCenter;
    gp_Pnt  smallCenter;
};

std::vector<ConeInfo> collectCones(const Workpiece& wp)
{
    std::vector<ConeInfo> out;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        BRepAdaptor_Surface surf(wp.face(fIdx));
        if (surf.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cone = surf.Cone();

        ConeInfo info;
        info.faceIdx      = fIdx;
        info.semiAngleRad = std::abs(cone.SemiAngle());
        info.axis         = cone.Axis();

        std::vector<std::pair<double, gp_Pnt>> circles;
        for (TopExp_Explorer exp(wp.face(fIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(info.axis.Direction())) - 1.0) > 1e-3)
                continue;
            circles.emplace_back(c.Radius(), c.Location());
        }
        if (circles.empty()) continue;

        // A spot's cone bottom degenerates: only ONE circular edge (the large
        // top) may be present, or two if OCCT emitted the tiny apex circle.
        // Handle both: pick max-radius circle as large end, min-radius as small.
        const auto maxR = std::max_element(circles.begin(), circles.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });
        const auto minR = std::min_element(circles.begin(), circles.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });
        info.rLarge      = maxR->first;
        info.largeCenter = maxR->second;
        info.rSmall      = minR->first;
        info.smallCenter = minR->second;
        out.push_back(info);
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cones = collectCones(wp);
    if (cones.empty()) return out;

    // Collect cylinder axes to disqualify cones that pair with a cylinder
    // (those are countersinks / counterbores / etc., not spot marks).
    std::vector<gp_Ax1> cylAxes;
    std::vector<double> cylRadii;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        cylAxes.push_back(surf.Cylinder().Axis());
        cylRadii.push_back(surf.Cylinder().Radius());
    }

    auto sharesAxisWithCyl = [&](const gp_Ax1& coneAxis, double coneRSmall) {
        for (size_t i = 0; i < cylAxes.size(); ++i) {
            // Axis parallelism check
            const double dot = std::abs(coneAxis.Direction().Dot(cylAxes[i].Direction()));
            if (dot < std::cos(2.0 * M_PI / 180.0)) continue;
            // Position: cone axis line passes near cylinder axis line.
            const gp_Pnt& pa = coneAxis.Location();
            const gp_Pnt& pb = cylAxes[i].Location();
            gp_Vec v(pa, pb);
            gp_Vec axV(coneAxis.Direction());
            gp_Vec perp = v - axV * v.Dot(axV);
            if (perp.Magnitude() > 1e-2) continue;
            // Cylinder radius approximately equals cone's small radius?  If
            // so the cone is the "lead-in" portion of a drill/countersink.
            if (std::abs(cylRadii[i] - coneRSmall) < 0.1) return true;
            // Otherwise still treat the presence of a coaxial cylinder as
            // disqualifying — spot drills do NOT pair with a bore.
            return true;
        }
        return false;
    };

    for (const auto& cone : cones) {
        // Disqualify cones that are part of a drill_hole/countersink combo.
        if (sharesAxisWithCyl(cone.axis, cone.rSmall)) continue;
        // Disqualify true frusta (small radius not near zero) — those are
        // chamfers or counterbore lead-ins, not spot marks.
        if (cone.rSmall > std::max(0.05, cone.rLarge * 0.10)) continue;

        const double dia = 2.0 * cone.rLarge;
        const double coneAngleDeg = 2.0 * cone.semiAngleRad * 180.0 / M_PI;
        const double halfRad = cone.semiAngleRad;
        const double t = std::tan(halfRad);
        const double depth = (t > 1e-9) ? (cone.rLarge / t) : 0.0;

        // Axis direction: from large circle (entry) into the material toward
        // the apex.  smallCenter sits inside; pick that direction.
        gp_Vec dirVec(cone.largeCenter, cone.smallCenter);
        if (dirVec.Magnitude() < 1e-9) {
            // Degenerate: small circle near apex collapsed → fall back to
            // cone axis direction (orientation ambiguous).
            dirVec = gp_Vec(cone.axis.Direction());
        }
        dirVec.Normalize();

        // Confidence — high if cone is shallow (depth/dia < 1) and the small
        // radius is near zero.
        double conf = 0.85;
        if (cone.rSmall > 0.05) conf -= 0.10;
        if (dia > 0.0 && depth / dia > 1.2) conf -= 0.15;  // unusually deep
        conf = std::clamp(conf, 0.3, 0.90);

        json recovered = {
            { "position_x_mm",   cone.largeCenter.X() },
            { "position_y_mm",   cone.largeCenter.Y() },
            { "axis_dir",        { dirVec.X(), dirVec.Y(), dirVec.Z() } },
            { "diameter_mm",     dia },
            { "cone_angle_deg",  coneAngleDeg },
            { "depth_mm",        depth },
        };
        json matched = {
            { "cone_face_id",    cone.faceIdx },
            { "entry_center",    { cone.largeCenter.X(), cone.largeCenter.Y(),
                                   cone.largeCenter.Z() } },
            { "apex_center",     { cone.smallCenter.X(), cone.smallCenter.Y(),
                                   cone.smallCenter.Z() } },
            { "r_small",         cone.rSmall },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::spot_drill
