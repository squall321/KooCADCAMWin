// @lat: [[engine/skills#spot_face]]

#include "spot_face.hpp"

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

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace koocadcam::skill::spot_face {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dia_mm < 3.0) {
        r.add("DFM-002", "error",
              "spot_face dia_mm " + std::to_string(in.dia_mm) +
              " < min 3.0 mm (spot-face cutter min)");
    }
    if (in.dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "dia_mm must be > 0");
    }
    if (in.depth_mm < 0.3) {
        r.add("DFM-SPOTFACE-DEPTH", "error",
              "spot_face depth_mm " + std::to_string(in.depth_mm) +
              " < 0.3 mm — insufficient flat seat depth");
    }
    if (in.depth_mm > 5.0) {
        r.add("DFM-SPOTFACE-DEPTH", "error",
              "spot_face depth_mm " + std::to_string(in.depth_mm) +
              " > 5.0 mm — deeper than a spot-face; use counterbore or "
              "mill_circular_pocket instead");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spot_face DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("spot_face: entry_face datum unresolved");

    // Identical to mill_circular_pocket apply with sharp corner (no fillet).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Entry point = axis∩entry-face plane (any axis; a prior bbox-margin guess
    // put the cutter a whole bboxDiag behind the part for a tilted axis).
    const gp_Pnt entryPt =
        entryPointOnFacePlane(wp, *entryId, in.position_x_mm, in.position_y_mm,
                              adir, zMin, zMax);
    const gp_Pnt toolStart(entryPt.X() - adir.X() * kOverhang,
                           entryPt.Y() - adir.Y() * kOverhang,
                           entryPt.Z() - adir.Z() * kOverhang);

    const double toolHeight = in.depth_mm + kOverhang;
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.dia_mm / 2.0, toolHeight);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "position_z_mm",  entryPt.Z() },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "dia_mm",         in.dia_mm },
        { "depth_mm",       in.depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", true },
        { "dia_mm",                     in.dia_mm },
        { "depth_mm",                   in.depth_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "intent",                     "flat_seat_for_fastener" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "spot_face_cutter";
    tooling.tool_dia_mm       = in.dia_mm;
    tooling.tool_length_mm    = in.depth_mm * 2.0 + 8.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.020;
    tooling.stock_removed_mm3 = M_PI * (in.dia_mm / 2.0) * (in.dia_mm / 2.0)
                                * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm * 0.8);  // shallow + slow
    tooling.extra["intent"] = "fastener_seat";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::spot_face applied: dia={} depth={} faces {}→{}",
                  in.dia_mm, in.depth_mm, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: SHALLOW circular pocket — same topology as mill_circular_pocket
// but with depth/dia < 0.5 (the spot-face sweet spot).  Recognition
// confidence 0.65; overlaps with mill_circular_pocket and counterbore
// (counterbore additionally has a smaller drill hole below the seat).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>,
        TopTools_ShapeMapHasher> ef;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, ef);

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();

        std::vector<gp_Pnt> circleCenters;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > 1e-2) continue;
            circleCenters.push_back(c.Location());
        }
        if (circleCenters.size() < 2) continue;

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
        const double dia   = 2.0 * radius;
        if (dia < 1e-6) continue;

        // Spot-face: shallow (depth/dia < 0.5) and dia ≥ 3 mm.
        const double ratio = depth / dia;
        if (ratio >= 0.5) continue;
        if (dia < 3.0)    continue;

        // Bottom must be a flat planar face ≈ π r².
        double minPlanarArea = std::numeric_limits<double>::max();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!ef.Contains(e)) continue;
            const auto& adj = ef.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                GProp_GProps gp;
                BRepGProp::SurfaceProperties(af, gp);
                minPlanarArea = std::min(minPlanarArea, gp.Mass());
            }
        }
        if (minPlanarArea == std::numeric_limits<double>::max()) continue;

        const double expectedBottom = M_PI * radius * radius;
        if (std::abs(minPlanarArea - expectedBottom) / expectedBottom > 0.25)
            continue;

        gp_Vec dirVec(centerHigh, centerLow);
        if (dirVec.Magnitude() < 1e-6) continue;
        dirVec.Normalize();

        json recovered = {
            { "position_x_mm", centerHigh.X() },
            { "position_y_mm", centerHigh.Y() },
            { "position_z_mm", centerHigh.Z() },   // entry Z (consistent with x/y) so a
                                                   // radial spot-face machines from the
                                                   // real surface, not Z=0
            { "axis_dir",      { dirVec.X(), dirVec.Y(), dirVec.Z() } },
            { "dia_mm",        dia },
            { "depth_mm",      depth },
        };
        json matched = {
            { "cylindrical_face_id",  fIdx },
            { "top_center",           { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bottom_center",        { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "aspect_ratio",         ratio },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::spot_face
