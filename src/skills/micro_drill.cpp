// @lat: [[engine/skills#micro_drill]]

#include "micro_drill.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <limits>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::micro_drill {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "micro_drill diameter must be > 0");
    }
    if (!in.through_hole && in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "micro_drill blind depth must be > 0");
    }

    // Sub-mm envelope: hard floor at 0.1 mm.
    if (in.diameter_mm > 0.0 && in.diameter_mm < 0.1) {
        r.add("DFM-MICRO-DIAMIN", "error",
              "micro_drill diameter " + std::to_string(in.diameter_mm) +
              " mm < min 0.1 mm — below practical limit");
    }

    // Above 0.8 mm: use regular drill_hole instead.  Warning, not error.
    if (in.diameter_mm >= 0.8) {
        r.add("DFM-MICRO-DIAMAX", "warning",
              "micro_drill diameter " + std::to_string(in.diameter_mm) +
              " mm ≥ 0.8 mm — outside the micro-drill regime; prefer drill_hole");
    }

    // Stiffness wall: micro-drills snap above depth/dia ≈ 5.
    const double ratio = (in.diameter_mm > 0.0 && in.depth_mm > 0.0)
                       ? (in.depth_mm / in.diameter_mm)
                       : 0.0;
    if (!in.through_hole && ratio > 5.0) {
        r.add("DFM-MICRO-RATIO", "error",
              "depth/dia ratio " + std::to_string(ratio) +
              " > 5 — micro-drill will snap; reduce depth or increase dia");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "micro_drill DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("micro_drill: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart(
        in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    }

    const double toolHeight = in.through_hole
        ? (bboxDiag + 2.0 * kEntryOverhang)
        : (in.depth_mm + kEntryOverhang);

    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",     in.diameter_mm },
        { "depth_mm",        in.depth_mm },
        { "through_hole",    in.through_hole },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", !in.through_hole },
        { "dia_mm",                     in.diameter_mm },
        { "diameter_mm",                in.diameter_mm },
        { "depth_mm",                   in.depth_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    // Micro-drill tooling: solid carbide, sub-mm, very high RPM, very low feed.
    ToolingMeta tooling;
    tooling.tool_type        = "micro_drill";
    tooling.tool_dia_mm      = in.diameter_mm;
    tooling.tool_length_mm   = in.depth_mm * 1.5 + 3.0;
    tooling.tool_material    = "carbide";   // solid carbide essential below 0.8
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 400.0;      // high SFM, but tiny dia → modest RPM × dia
    tooling.feed_per_tooth_mm = 0.005;      // sub-0.01 — extreme care
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                              * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(3.0, in.depth_mm / 5.0);

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::micro_drill applied: dia={} depth={} faces {}→{}",
                  in.diameter_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy: drill_hole topology + diameter < 0.8.  Confidence 0.85 because
// the diameter envelope cleanly differentiates micro_drill from drill_hole
// (regular drill_hole has dia ≥ 0.8 by DFM-002).

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

        // Micro regime — radius < 0.4 (i.e. dia < 0.8).
        const double diameter = 2.0 * radius;
        if (diameter >= 0.8) continue;
        if (diameter < 0.1)  continue;   // below practical limit

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
        const auto minIt = std::min_element(circleCenters.begin(), circleCenters.end(), cmp);
        const auto maxIt = std::max_element(circleCenters.begin(), circleCenters.end(), cmp);
        const gp_Pnt centerLow  = *minIt;
        const gp_Pnt centerHigh = *maxIt;
        const double depth = centerHigh.Distance(centerLow);

        gp_Vec drillVec(centerHigh, centerLow);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        // Blind / through detection.
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

        json recovered = {
            { "position_x_mm", centerHigh.X() },
            { "position_y_mm", centerHigh.Y() },
            { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",   diameter },
            { "depth_mm",      through ? 0.0 : depth },
            { "through_hole",  through },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center", { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bot_center", { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "dia_mm",  diameter },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.85, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::micro_drill
