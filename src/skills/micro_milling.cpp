// @lat: [[engine/skills#micro_milling]]

#include "micro_milling.hpp"

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

namespace koocadcam::skill::micro_milling {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "micro_milling: diameter_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "micro_milling: depth_mm must be > 0");
    }

    // Sub-millimetre envelope: hard floor at 0.05 mm.
    if (in.diameter_mm > 0.0 && in.diameter_mm < 0.05) {
        r.add("DFM-MICRO-MILL-DIAMIN", "error",
              "micro_milling: diameter " + std::to_string(in.diameter_mm) +
              " mm < min 0.05 mm — below practical micro-mill limit");
    }
    // Above 0.5 mm: regular mill_circular_pocket is the right tool.
    if (in.diameter_mm > 0.5) {
        r.add("DFM-MICRO-MILL-DIAMAX", "error",
              "micro_milling: diameter " + std::to_string(in.diameter_mm) +
              " mm > 0.5 mm — outside the micro-mill regime; "
              "use mill_circular_pocket");
    }

    // Stiffness wall: micro end-mill deflects beyond tolerance OR snaps
    // above depth/dia ≈ 5.
    const double ratio = (in.diameter_mm > 0.0 && in.depth_mm > 0.0)
                       ? (in.depth_mm / in.diameter_mm)
                       : 0.0;
    if (ratio > 5.0) {
        r.add("DFM-MICRO-MILL-RATIO", "error",
              "micro_milling: depth/dia ratio " + std::to_string(ratio) +
              " > 5 — micro-mill will deflect / snap; reduce depth or "
              "increase diameter");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "micro_milling DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("micro_milling: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kEntryOverhang = 0.01;   // tighter than regular pocket — micro feature
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    } else {
        // Generic fallback for non-axial micro mills.
        const double bboxDiag = std::sqrt(
            (xMax - xMin)*(xMax - xMin) +
            (yMax - yMin)*(yMax - yMin) +
            (zMax - zMin)*(zMax - zMin));
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax)/2.0 - adir.Z() * margin);
    }

    const double toolHeight = in.depth_mm + kEntryOverhang;
    // gp_Ax2 V = local Z (drill direction in axis_dir) — see header pitfall note.
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_kind",      "resolved_id" },
        { "entry_face_id",        *entryId },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",          in.diameter_mm },
        { "depth_mm",             in.depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "toroidal_face_count",        0 },
        { "bottom_planar_face_present", true },
        { "circular_edge_count",        2 },
        { "diameter_mm",                in.diameter_mm },
        { "depth_mm",                   in.depth_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    // Micro-milling tooling: solid carbide micro end-mill, high RPM, very low feed.
    ToolingMeta tooling;
    tooling.tool_type         = "micro_end_mill";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 3.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;                // 2-flute for chip clearance at sub-mm
    tooling.cutting_speed_sfm = 500.0;            // higher SFM, tiny dia → high RPM
    tooling.feed_per_tooth_mm = 0.003;            // sub-0.01 — extreme care
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm/2.0) * (in.diameter_mm/2.0)
                              * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(2.0, in.depth_mm * 1.5 + in.diameter_mm * 1.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::micro_milling applied: dia={} depth={} faces {}→{}",
                  in.diameter_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: cylindrical face whose diameter is in the micro-milling envelope
// [0.05, 0.5] mm AND adjacent to a small planar bottom face (NOT a through
// drill).  Use aspect ratio < 2 as the "pocket vs hole" gate to avoid
// claiming micro_drill features.

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

        // Diameter envelope filter: [0.05, 0.5] mm.
        const double diameter = 2.0 * radius;
        if (diameter < 0.05) continue;
        if (diameter > 0.5)  continue;

        // Collect bounding circles.
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
        if (depth < 1e-4) continue;

        // Must have an adjacent small planar bottom (NOT a through hole).
        const double bottomArea = M_PI * radius * radius;
        bool hasSmallPlanarBottom = false;
        double smallestAdjPlanarArea = std::numeric_limits<double>::max();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                BRepAdaptor_Surface as(af);
                if (as.GetType() != GeomAbs_Plane) continue;
                GProp_GProps gp;
                BRepGProp::SurfaceProperties(af, gp);
                smallestAdjPlanarArea = std::min(smallestAdjPlanarArea, gp.Mass());
                hasSmallPlanarBottom = true;
            }
        }
        if (!hasSmallPlanarBottom) continue;
        // Bottom must be near π r² (not the workpiece outer face).
        if (smallestAdjPlanarArea > bottomArea * 1.5) continue;

        // Pocket aspect ratio: depth/dia < 5 differentiates from micro_drill.
        // (Originally < 2.0, but legitimate micro-milled blind pockets often
        // have depth/dia up to the DFM-MICRO-MILL-RATIO ceiling of 5.)
        const double ratio = (diameter > 0.0) ? depth / diameter : 0.0;
        if (ratio > 5.0) continue;

        gp_Vec drillVec(centerHigh, centerLow);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        json recovered = {
            { "position_x_mm", centerHigh.X() },
            { "position_y_mm", centerHigh.Y() },
            { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",   diameter },
            { "depth_mm",      depth },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center",          { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bottom_center",       { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "bottom_area_mm2",     smallestAdjPlanarArea },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.85, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::micro_milling
