// @lat: [[engine/skills#abrasive_water_jet_drill]]

#include "abrasive_water_jet_drill.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <limits>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::abrasive_water_jet_drill {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// AWJD process envelope (shared with `_separation_common` waterjet limits but
// expressed per single-hole here):
//   - diameter ∈ [0.5, 25] mm  (DFM-002)
//   - thickness ≤ 200 mm        (DFM-THICKNESS)
//   - finish "smooth", no HAZ   (DFM-SURFACE)
constexpr double kDiameterMinMm    = 0.5;
constexpr double kDiameterMaxMm    = 25.0;
constexpr double kMaxThicknessMm   = 200.0;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm < kDiameterMinMm || in.diameter_mm > kDiameterMaxMm) {
        r.add("DFM-002", "error",
              std::string(kSkillId) + ": diameter_mm " +
              std::to_string(in.diameter_mm) +
              " outside AWJD range [" + std::to_string(kDiameterMinMm) +
              ", " + std::to_string(kDiameterMaxMm) + "] mm");
    }
    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": diameter_mm must be > 0");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > kMaxThicknessMm) {
        r.add("DFM-THICKNESS", "error",
              std::string(kSkillId) + ": workpiece thickness " +
              std::to_string(thickness) + " mm > process max " +
              std::to_string(kMaxThicknessMm) + " mm");
    }

    r.add("DFM-SURFACE", "info",
          std::string(kSkillId) + ": expected bore finish = smooth (no HAZ)");

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "abrasive_water_jet_drill DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("abrasive_water_jet_drill: entry_face datum unresolved");

    // AWJD is ALWAYS through-cut: build a cylinder that fully spans the
    // workpiece in axis_dir, with a small overhang above + below for clean
    // Boolean booleans.  Only the simple (x,y,z) → -Z direction is taken on
    // the fast path; off-axis drilling falls back to a bbox-diagonal sweep
    // identical to drill_hole.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    constexpr double kOverhang = 0.5;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart(0.0, 0.0, 0.0);
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        // Fast path: Z-axis drill.
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                               zMax + kOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                               zMin - kOverhang);
        }
    } else {
        // Generic path: start far outside along -axis_dir.
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * (margin + kOverhang),
            in.position_y_mm - adir.Y() * (margin + kOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (margin + kOverhang));
    }

    const double toolHeight = bboxDiag + 2.0 * kOverhang;

    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",     in.diameter_mm },
    };

    const double thickness = zMax - zMin;

    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", false },
        { "diameter_mm",                in.diameter_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "through_hole",               true },
        { "tool_type",                  "waterjet" },
        { "surface_finish",             "smooth" },
        { "process_category",           "separation_drilling" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "waterjet";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = thickness;
    tooling.tool_material     = "garnet_abrasive";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) *
                                (in.diameter_mm / 2.0) * thickness;
    // AWJD pierce time ≈ thickness / 5 (mm/s in 25 mm steel; slower thicker).
    tooling.est_cycle_time_s  = std::max(1.0, thickness / 5.0);
    tooling.extra = {
        { "surface_finish",     "smooth" },
        { "haz",                "none" },
        { "process_category",   "separation_drilling" },
        { "dfm_findings",       findings },
        { "machining_constraint",
          "AWJD — smooth bore, no HAZ, ideal for composites / ceramics" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::abrasive_water_jet_drill applied: dia={} thickness={} faces {}→{}",
                  in.diameter_mm, thickness,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Identical topology to drill_hole through-holes (1 cylindrical face + 2
// circular edges, no bottom planar face).  We mirror drill_hole's scanner
// inline (not via #include drill_hole) to keep the dependency surface
// narrow, and emit AWJD candidates only when 0.5 ≤ dia ≤ 25 mm AND the hole
// is through-cut.  Confidence 0.55 — overlaps strongly with drill_hole so
// caller must disambiguate (e.g. workpiece material = composite → AWJD).

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    const auto edgeFaces = buildEdgeFaceMap(shape);

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const gp_Ax1 axis = cyl.Axis();
        const double dia = 2.0 * radius;

        // AWJD diameter band: [0.5, 25] mm.
        if (dia < kDiameterMinMm || dia > kDiameterMaxMm) continue;

        // Collect circular edges on this cylinder.
        std::vector<gp_Pnt> circleCenters;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(axis.Direction())) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            circleCenters.push_back(c.Location());
        }
        if (circleCenters.size() < 2) continue;

        const gp_Dir adir = axis.Direction();
        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - axis.Location().X()) * adir.X() +
                   (p.Y() - axis.Location().Y()) * adir.Y() +
                   (p.Z() - axis.Location().Z()) * adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) {
            return projOnAxis(a) < projOnAxis(b);
        };
        const auto minIt = std::min_element(circleCenters.begin(),
                                            circleCenters.end(), cmp);
        const auto maxIt = std::max_element(circleCenters.begin(),
                                            circleCenters.end(), cmp);
        const gp_Pnt centerLow  = *minIt;
        const gp_Pnt centerHigh = *maxIt;

        // Through-hole detection identical to drill_hole — if any adjacent
        // planar face has area close to π r², it's a blind bottom and AWJD
        // (always through) does NOT match.
        const double drillBottomArea = M_PI * radius * radius;
        double minAdjPlanarArea = std::numeric_limits<double>::max();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                GProp_GProps gp;
                BRepGProp::SurfaceProperties(af, gp);
                minAdjPlanarArea = std::min(minAdjPlanarArea, gp.Mass());
            }
        }
        const bool through = (minAdjPlanarArea > drillBottomArea * 1.5);
        if (!through) continue;   // AWJD is always through-cut.

        gp_Vec drillVec(centerHigh, centerLow);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        json recovered = {
            { "position_x_mm", centerHigh.X() },
            { "position_y_mm", centerHigh.Y() },
            { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",   dia },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center", { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bot_center", { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "through_hole", true },
        };
        // Confidence 0.55 — geometric topology is shared with drill_hole, so
        // disambiguation requires extra context (material, surface finish).
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.55, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::abrasive_water_jet_drill
