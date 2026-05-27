// @lat: [[engine/skills#mill_circular_pocket]]

#include "mill_circular_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Fillets.hpp"
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

namespace koocadcam::skill::mill_circular_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm < 0.8) {
        r.add("DFM-002", "error",
              "mill_circular_pocket diameter " + std::to_string(in.diameter_mm) +
              " mm < min 0.8 mm (end-mill min size)");
    }
    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "diameter must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "depth must be > 0");
    }
    if (in.bottom_corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error", "bottom_corner_r must be >= 0");
    }
    if (in.bottom_corner_r_mm > 0.0 && in.bottom_corner_r_mm < 0.2) {
        r.add("DFM-004", "error",
              "bottom_corner_r " + std::to_string(in.bottom_corner_r_mm) +
              " mm < min R 0.2 mm");
    }
    if (in.bottom_corner_r_mm > 0.0 &&
        in.bottom_corner_r_mm > in.diameter_mm / 2.0 - 1e-6) {
        r.add("DFM-INPUT", "error",
              "bottom_corner_r must be < radius (diameter/2)");
    }
    if (in.bottom_corner_r_mm > 0.0 &&
        in.bottom_corner_r_mm > in.depth_mm - 1e-6) {
        r.add("DFM-INPUT", "error",
              "bottom_corner_r must be < depth");
    }

    const double ratio = (in.diameter_mm > 0.0) ? (in.depth_mm / in.diameter_mm) : 0.0;
    if (ratio > 5.0) {
        r.add("DFM-AR", "warning",
              "depth/dia ratio " + std::to_string(ratio) +
              " > 5 — pocket too deep for end-mill stiffness");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mill_circular_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mill_circular_pocket: entry_face datum unresolved");

    // Build the cutter cylinder.  Cutter starts slightly above the entry face
    // (along -axis_dir) and extends `depth + kEntryOverhang` along axis_dir.
    // The bottom of the cylinder lies at exactly `depth_mm` below the entry
    // plane → flat bottom.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Resolve a Z for the entry plane: project bbox along axis_dir.
    // For the common Z-aligned drill-down case (axis = -Z) start at zMax.
    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    } else {
        // Fallback: position the tool start on the bbox boundary along -axis_dir
        // from the requested XY.  For non-axial pockets, the caller is
        // responsible for ensuring this places the tool above the entry face.
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
    const gp_Ax2 toolAx(toolStart, adir);
    TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    // Bottom corner fillet on the CUTTER: the cutter's bottom-edge is a
    // single circle of radius `diameter/2`.  Filleting that edge by
    // `bottom_corner_r` produces a torus blend at the bottom of the cutter,
    // which after Boolean cut becomes the bottom-corner fillet in the pocket.
    if (in.bottom_corner_r_mm > 0.0) {
        // The bottom edge of the cutter sits at parametric "bottom" of the
        // cylinder.  In world coords (for the axial top-down case) that's at
        // Z = toolStart.Z() + adir.Z() * toolHeight.
        const double zBottom = toolStart.Z() + adir.Z() * toolHeight;
        cutter = pr::filletEdges(cutter, in.bottom_corner_r_mm,
            [zBottom](const TopoDS_Edge& e, const gp_Pnt& mp) {
                (void)e;
                return std::abs(mp.Z() - zBottom) < 1e-3;
            });
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Build signature.
    json params = {
        { "entry_face_kind",      "resolved_id" },
        { "entry_face_id",        *entryId },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",          in.diameter_mm },
        { "depth_mm",             in.depth_mm },
        { "bottom_corner_r_mm",   in.bottom_corner_r_mm },
    };
    const bool hasFillet = in.bottom_corner_r_mm > 0.0;
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     hasFillet ? 1 : 1 },
        { "toroidal_face_count",        hasFillet ? 1 : 0 },
        { "bottom_planar_face_present", true },
        { "circular_edge_count",        hasFillet ? 2 : 2 },
        { "diameter_mm",                in.diameter_mm },
        { "depth_mm",                   in.depth_mm },
        { "bottom_corner_r_mm",         in.bottom_corner_r_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 600.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm/2.0) * (in.diameter_mm/2.0)
                                * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm * 0.5 + in.diameter_mm * 0.1);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::mill_circular_pocket applied: dia={} depth={} cornerR={} faces {}→{}",
                  in.diameter_mm, in.depth_mm, in.bottom_corner_r_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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
        const double radius   = cyl.Radius();
        const gp_Ax1   axis   = cyl.Axis();
        const gp_Dir   adir   = axis.Direction();

        // Collect circular edges that bound the cylinder, taking only the
        // two extreme circles (top + bottom) along the cyl axis.
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

        // We need TWO planar adjacent faces:
        //   - the large planar (the entry face — the workpiece top)
        //   - the small planar (the pocket bottom) of area ≈ π r² OR a
        //     smaller area if there's a bottom-corner fillet (in which case
        //     a toroidal adjacent face also exists).
        //
        // If the smallest adjacent planar is NOT close to π r² we skip
        // (it may be a drill_hole's blind bottom which is exactly π r²
        // anyway — we use ASPECT RATIO to distinguish drill from pocket).
        bool hasSmallPlanarBottom = false;
        bool hasToroidalAdjacent  = false;
        double smallestAdjPlanarArea = std::numeric_limits<double>::max();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                BRepAdaptor_Surface as(af);
                if (as.GetType() == GeomAbs_Plane) {
                    GProp_GProps gp;
                    BRepGProp::SurfaceProperties(af, gp);
                    smallestAdjPlanarArea = std::min(smallestAdjPlanarArea, gp.Mass());
                    hasSmallPlanarBottom = true;
                } else if (as.GetType() == GeomAbs_Torus) {
                    hasToroidalAdjacent = true;
                }
            }
        }
        if (!hasSmallPlanarBottom) continue;

        // Distinguish from drill_hole by aspect ratio: pockets are wide
        // (depth/dia < 2).  Above 2, defer to drill_hole skill.
        const double dia = 2.0 * radius;
        const double ratio = (dia > 0.0) ? depth / dia : 0.0;
        if (ratio >= 2.0) continue;

        // The expected pocket-bottom area is π r² (sharp corner) or
        // π (r - rCorner)² + toroidal area for the fillet case.  We don't
        // try to invert this exactly — we just check "small planar exists".
        const double expectedSharpBottom = M_PI * radius * radius;
        // Confidence drops if planar area is far off the disc-area expectation.
        const double areaErr = std::abs(smallestAdjPlanarArea - expectedSharpBottom)
                             / std::max(1e-6, expectedSharpBottom);
        const double conf = std::clamp(0.90 - 0.5 * std::min(areaErr, 1.0), 0.4, 0.95);

        // Recover bottom_corner_r heuristically: if we saw a torus, look it up.
        double recoveredCornerR = 0.0;
        if (hasToroidalAdjacent) {
            for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
                const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
                if (!edgeFaces.Contains(e)) continue;
                const auto& adj = edgeFaces.FindFromKey(e);
                for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                    const TopoDS_Face& af = TopoDS::Face(it.Value());
                    if (af.IsSame(cylFace)) continue;
                    BRepAdaptor_Surface as(af);
                    if (as.GetType() == GeomAbs_Torus) {
                        recoveredCornerR = as.Torus().MinorRadius();
                        break;
                    }
                }
                if (recoveredCornerR > 0.0) break;
            }
        }

        // drill direction = from high → low (entry → bottom)
        gp_Vec drillVec(centerHigh, centerLow);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        json recovered = {
            { "position_x_mm",       centerHigh.X() },
            { "position_y_mm",       centerHigh.Y() },
            { "axis_dir",            { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",         dia },
            { "depth_mm",            depth },
            { "bottom_corner_r_mm",  recoveredCornerR },
        };
        json matched = {
            { "cylindrical_face_id",   fIdx },
            { "top_center",            { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bottom_center",         { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "bottom_planar_area",    smallestAdjPlanarArea },
            { "has_torus_blend",       hasToroidalAdjacent },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mill_circular_pocket
