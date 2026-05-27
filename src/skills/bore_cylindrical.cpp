// @lat: [[engine/skills#bore_cylindrical]]

#include "bore_cylindrical.hpp"

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

namespace koocadcam::skill::bore_cylindrical {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_cylindrical diameter must be > 0");
    }
    if (in.diameter_mm < 0.8) {
        r.add("DFM-002", "error",
              "bore_cylindrical diameter " + std::to_string(in.diameter_mm) +
              " mm < min 0.8 mm");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_cylindrical depth must be > 0");
    }

    // Boring-bar specific: usually 6 mm and up.  Smaller diameters are
    // technically possible with a small-bore tool but typically reach H7
    // via drill + reamer instead.
    if (in.diameter_mm > 0.0 && in.diameter_mm < 6.0) {
        r.add("DFM-BORE-DIA", "warning",
              "bore_cylindrical diameter " + std::to_string(in.diameter_mm) +
              " mm < 6 mm — boring bars are usually 6 mm+; consider drill+ream");
    }

    // Boring-bar deflection: depth/dia > 4 causes chatter and taper.
    const double ratio = (in.diameter_mm > 0.0) ? (in.depth_mm / in.diameter_mm) : 0.0;
    if (ratio > 4.0) {
        r.add("DFM-BORE-RATIO", "warning",
              "depth/dia ratio " + std::to_string(ratio) +
              " > 4 — boring bar deflection may degrade tolerance");
    }

    // Tolerance class validation
    if (!in.tolerance_class.empty() &&
        in.tolerance_class != "H7" &&
        in.tolerance_class != "H8" &&
        in.tolerance_class != "H9") {
        r.add("DFM-INPUT", "warning",
              "unknown tolerance_class '" + in.tolerance_class +
              "' — expected H7/H8/H9");
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
        std::string msg = "bore_cylindrical DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Resolve entry face datum
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("bore_cylindrical: entry_face datum unresolved");

    // 3) Compute bore geometry & start point
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Default start: pull back along reversed axis from the bbox centre.
    gp_Pnt toolStart(
        in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));

    // Simple axial straight-down/up case
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    }

    const double toolHeight = in.depth_mm + kEntryOverhang;

    // 4) Build cutter
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    // 5) Cut
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // 6) Signature
    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",     in.diameter_mm },
        { "depth_mm",        in.depth_mm },
        { "tolerance_class", in.tolerance_class },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", true },
        { "diameter_mm",                in.diameter_mm },
        { "depth_mm",                   in.depth_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "tolerance_class",            in.tolerance_class },
    };

    // Boring uses slower SFM than drilling.  Carbide insert, single point.
    ToolingMeta tooling;
    tooling.tool_type        = "boring_bar";
    tooling.tool_dia_mm      = in.diameter_mm;
    tooling.tool_length_mm   = in.depth_mm * 1.5 + 10.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 1;             // single-point boring tool
    tooling.cutting_speed_sfm = 100.0;        // slower than drilling
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                              * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(2.0, in.depth_mm / 25.0);
    tooling.extra = json{
        { "tolerance_class", in.tolerance_class },
    };

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::bore_cylindrical applied: dia={} depth={} tol={} faces {}→{}",
                  in.diameter_mm, in.depth_mm, in.tolerance_class,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy mirrors drill_hole::recognize, but uses recovered metrics to
// decide whether the topology represents a bore (dia ≥ 6 mm AND
// depth/dia ≤ 4) versus a drilled hole.  Confidence is high when both
// criteria match, low when only one does (ambiguous), and the candidate
// is skipped when neither matches.

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

        // Gather the two extreme circular edges on this cylinder.
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
        const double diameter = 2.0 * radius;

        gp_Vec drillVec(centerHigh, centerLow);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        // Through-vs-blind detection (same heuristic as drill_hole).
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

        // Bore signature: blind, precision diameter, low aspect ratio.
        if (through) continue;  // bore_cylindrical synthesis is blind
        const double ratio = (diameter > 0.0) ? (depth / diameter) : 0.0;
        const bool diaOk   = diameter >= 6.0;
        const bool ratioOk = ratio <= 4.0;
        if (!diaOk && !ratioOk) continue;

        // Confidence:
        //   both criteria → 0.9 (strong bore signature)
        //   only diameter → 0.55 (could be a shallow drill)
        //   only ratio    → 0.45 (small dia, but shallow — likely drill+ream)
        double confidence;
        if (diaOk && ratioOk)      confidence = 0.9;
        else if (diaOk)            confidence = 0.55;
        else                       confidence = 0.45;

        json recovered = {
            { "position_x_mm",   centerHigh.X() },
            { "position_y_mm",   centerHigh.Y() },
            { "axis_dir",        { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",     diameter },
            { "depth_mm",        depth },
            { "tolerance_class", "H7" },   // default precision class
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center", { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bot_center", { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "depth_dia_ratio", ratio },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, confidence, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::bore_cylindrical
