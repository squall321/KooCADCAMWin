// @lat: [[engine/skills#edm_drill]]

#include "edm_drill.hpp"

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
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace koocadcam::skill::edm_drill {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "edm_drill diameter_mm must be > 0");
    }
    if (!in.through_hole && in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "edm_drill blind depth_mm must be > 0");
    }
    if (in.diameter_mm > 0.0 && in.diameter_mm < kMinDia_mm) {
        r.add("DFM-EDMD-DIA-LO", "error",
              "edm_drill diameter_mm " + std::to_string(in.diameter_mm) +
              " < min " + std::to_string(kMinDia_mm) +
              " mm (electrode supply impractical)");
    }
    if (in.diameter_mm > kMaxDia_mm) {
        r.add("DFM-EDMD-DIA-HI", "error",
              "edm_drill diameter_mm " + std::to_string(in.diameter_mm) +
              " > max " + std::to_string(kMaxDia_mm) +
              " mm (use sinker EDM or drill_hole)");
    }
    if (in.diameter_mm > 0.0 && in.depth_mm > 0.0) {
        const double ratio = in.depth_mm / in.diameter_mm;
        if (ratio > kMaxRatio) {
            r.add("DFM-EDMD-RATIO", "error",
                  "edm_drill depth/dia ratio " + std::to_string(ratio) +
                  " > " + std::to_string(kMaxRatio) +
                  ":1 envelope — electrode runout / flush failure");
        } else if (ratio > 30.0) {
            r.add("DFM-EDMD-RATIO-WARN", "warning",
                  "edm_drill depth/dia ratio " + std::to_string(ratio) +
                  " > 30 — flush pressure / cycle time concerns");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "edm_drill DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("edm_drill: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    } else {
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax) / 2.0 - adir.Z() * margin);
    }

    const double toolHeight = in.through_hole
        ? (bboxDiag + 2.0 * kEntryOverhang)
        : (in.depth_mm + kEntryOverhang);

    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double ratio = (in.diameter_mm > 0.0) ? in.depth_mm / in.diameter_mm : 0.0;

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",    in.diameter_mm },
        { "depth_mm",       in.depth_mm },
        { "through_hole",   in.through_hole },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", !in.through_hole },
        { "diameter_mm",                in.diameter_mm },
        { "depth_to_dia_ratio",         ratio },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "edm_tube_electrode";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = in.depth_mm + 10.0;     // guide-bush stickout
    tooling.tool_material     = "brass";                // hollow tube
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                              * in.depth_mm;
    // EDM hole-popper feed ≈ 2 mm/min for small holes; cycle = depth / 2.
    tooling.est_cycle_time_s  = std::max(5.0, in.depth_mm * 30.0);
    tooling.extra["dfm_findings"] = findings;
    tooling.extra["machining_constraint"] =
        "Small-hole EDM — depth/dia up to 50:1, hollow tube electrode with "
        "internal flush, conductive workpiece required";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::edm_drill applied: dia={} depth={} ratio={} faces {}→{}",
                  in.diameter_mm, in.depth_mm, ratio,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometry: 1 cylindrical face + 2 circular edges (same as drill_hole).
// Discriminator: edm_drill works in a much narrower diameter / aspect-ratio
// envelope.  We claim a cylindrical bore as edm_drill iff:
//   (a) diameter ≤ kMaxDia_mm (1.0 mm)
//   (b) depth / diameter > 8 (above the twist-drill comfort zone)
// Otherwise let drill_hole own it.

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

    // Metadata replay path — exact match by signature.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric path — cylinder + circular edges within the EDM envelope.
    const TopoDS_Shape& shape = wp.shape();
    const auto edgeFaces = buildEdgeFaceMap(shape);

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius   = cyl.Radius();
        const double dia      = 2.0 * radius;
        if (dia > kMaxDia_mm) continue;     // not EDM-drill territory

        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();

        std::vector<gp_Pnt> circleCenters;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3) continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
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
        if (dia <= 0.0) continue;
        const double ratio = depth / dia;
        if (ratio <= 8.0) continue;         // belongs to drill_hole / pocket

        // through vs blind: smallest adjacent planar area test.
        const double drillBottomArea = M_PI * radius * radius;
        double minAdjPlanarArea = std::numeric_limits<double>::max();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
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

        gp_Vec drillVec(centerHigh, centerLow);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        json recovered = {
            { "position_x_mm", centerHigh.X() },
            { "position_y_mm", centerHigh.Y() },
            { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",   dia },
            { "depth_mm",      through ? 0.0 : depth },
            { "through_hole",  through },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "depth_to_dia_ratio",  ratio },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.85, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::edm_drill
