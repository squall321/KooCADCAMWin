// @lat: [[engine/skills#plunge_milling]]

#include "plunge_milling.hpp"

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

namespace koocadcam::skill::plunge_milling {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "plunge_milling dia_mm " + std::to_string(in.dia_mm) +
              " < min 0.8 mm (end-mill min size)");
    }
    if (in.dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "dia_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "depth_mm must be > 0");
    }
    const double ratio = (in.dia_mm > 0.0) ? (in.depth_mm / in.dia_mm) : 0.0;
    if (ratio > 3.0) {
        r.add("DFM-PLUNGE-AR", "error",
              "depth/dia ratio " + std::to_string(ratio) +
              " > 3.0 — plunge milling max safe ratio is 3 (use drill_hole "
              "for deeper narrow features)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "plunge_milling DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("plunge_milling: entry_face datum unresolved");

    // Build a flat-bottom end-mill cylinder.  Identical topology to
    // mill_circular_pocket with bottom_corner_r_mm = 0 — re-implementing
    // inline (rather than calling mill_circular_pocket::apply) to keep the
    // signature.pattern.kind and tooling metadata distinct.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
        }
    } else {
        const double bboxDiag = std::sqrt(
            (xMax - xMin)*(xMax - xMin) +
            (yMax - yMin)*(yMax - yMin) +
            (zMax - zMin)*(zMax - zMin));
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax) / 2.0 - adir.Z() * margin);
    }

    const double toolHeight = in.depth_mm + kOverhang;
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.dia_mm / 2.0, toolHeight);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
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
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = in.dia_mm;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;  // plunge-capable end-mills usually 2-flute
    tooling.cutting_speed_sfm = 350.0;  // lower than side-cutting
    tooling.feed_per_tooth_mm = 0.015;  // axial plunge is slower
    tooling.stock_removed_mm3 = M_PI * (in.dia_mm / 2.0) * (in.dia_mm / 2.0)
                                * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm / 30.0);  // slower than drill
    tooling.extra["cutting_mode"] = "plunge_only";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::plunge_milling applied: dia={} depth={} faces {}→{}",
                  in.dia_mm, in.depth_mm, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: 1 cylindrical face + flat planar bottom (area ≈ π r²) + 2 circular
// edges + aspect ratio depth/dia ≤ 3.  Overlaps with mill_circular_pocket
// and drill_hole topologically; differentiate by aspect ratio.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();

        // Collect the bounding circular edges of this cylinder.
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

        // Get extremes along cyl axis.
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
        const double ratio = depth / dia;
        // Plunge-milling claim: depth/dia ≤ 3.  Drill_hole owns deeper ratios.
        if (ratio > 3.0) continue;

        // Bottom must be FLAT (planar, area ≈ π r²).  Walk adjacent planar
        // faces of each circular edge via a global edge-face map.
        double minPlanarArea = std::numeric_limits<double>::max();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>,
            TopTools_ShapeMapHasher> ef;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, ef);
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
        // Sharp flat bottom ≈ π r², within 20 %.
        if (std::abs(minPlanarArea - expectedBottom) / expectedBottom > 0.20)
            continue;

        // drill direction = from high → low (entry → bottom)
        gp_Vec drillVec(centerHigh, centerLow);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        json recovered = {
            { "position_x_mm", centerHigh.X() },
            { "position_y_mm", centerHigh.Y() },
            { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "dia_mm",        dia },
            { "depth_mm",      depth },
        };
        json matched = {
            { "cylindrical_face_id",  fIdx },
            { "top_center",           { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bottom_center",        { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "bottom_planar_area",   minPlanarArea },
            { "aspect_ratio",         ratio },
        };
        // Confidence ~0.7 — overlaps with mill_circular_pocket on flat-bottom
        // narrow features.  Slightly higher when depth/dia is near the plunge
        // sweet spot (1–2).
        const double conf = (ratio >= 1.0 && ratio <= 2.5) ? 0.70 : 0.55;
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::plunge_milling
