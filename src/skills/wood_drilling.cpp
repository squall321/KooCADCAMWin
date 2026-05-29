// @lat: [[engine/skills#wood_drilling]]

#include "wood_drilling.hpp"

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

namespace koocadcam::skill::wood_drilling {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

bool isValidBitType(const std::string& s)
{
    return s == "brad_point" || s == "spade" ||
           s == "forstner"   || s == "auger";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": diameter_mm must be > 0");
    }
    if (!in.through_hole && in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": blind drill depth must be > 0");
    }
    if (in.diameter_mm > 0.0 && in.diameter_mm < 3.0) {
        r.add("DFM-WOOD-DIA-MIN", "error",
              std::string(kSkillId) + ": diameter_mm " +
              std::to_string(in.diameter_mm) +
              " < 3 mm — sub-3mm wood bits are fragile; use HSS twist drill");
    }
    if (in.diameter_mm > 50.0) {
        r.add("DFM-WOOD-DIA-MAX", "warning",
              std::string(kSkillId) + ": diameter_mm " +
              std::to_string(in.diameter_mm) +
              " > 50 mm — beyond typical Forstner / auger range; verify bit");
    }
    if (in.diameter_mm > 0.0 && in.depth_mm > 0.0) {
        const double ratio = in.depth_mm / in.diameter_mm;
        const bool isAuger = (in.bit_type == "auger");
        const double limit = isAuger ? 20.0 : 10.0;
        if (ratio > limit) {
            r.add("DFM-WOOD-DEPTH", "warning",
                  std::string(kSkillId) + ": depth/dia ratio " +
                  std::to_string(ratio) + " > " + std::to_string(limit) +
                  " — auger bit recommended for deep holes");
        }
    }
    if (in.end_grain) {
        r.add("DFM-WOOD-ENDGRAIN", "info",
              std::string(kSkillId) +
              ": drilling end-grain — back the work with sacrificial board to avoid tear-out");
    }
    if (!isValidBitType(in.bit_type)) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": bit_type '" + in.bit_type +
              "' unknown — expected 'brad_point' | 'spade' | 'forstner' | 'auger'");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wood_drilling DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("wood_drilling: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Same start-point convention as drill_hole.
    gp_Pnt toolStart(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    } else {
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * (margin + kEntryOverhang),
            in.position_y_mm - adir.Y() * (margin + kEntryOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (margin + kEntryOverhang));
    }

    const double toolHeight = in.through_hole
        ? (bboxDiag + 2.0 * kEntryOverhang)
        : (in.depth_mm + kEntryOverhang);

    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id", *entryId },
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "axis_dir",      { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",   in.diameter_mm },
        { "depth_mm",      in.depth_mm },
        { "through_hole",  in.through_hole },
        { "bit_type",      in.bit_type },
        { "end_grain",     in.end_grain },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", !in.through_hole },
        { "diameter_mm",                in.diameter_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "bit_type",                   in.bit_type },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "wood_drill_bit";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 10.0;
    tooling.tool_material     = "high_carbon_steel";
    tooling.flute_count       = (in.bit_type == "auger") ? 1 : 2;
    tooling.cutting_speed_sfm = 600.0;   // softwood
    tooling.feed_per_tooth_mm = 0.25;
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                              * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm / 100.0);  // wood drills faster
    tooling.extra["bit_type"] = in.bit_type;
    tooling.extra["machining_constraint"] =
        "Wood drilling — high RPM, low feed; clear chips frequently for deep holes.";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::wood_drilling applied: dia={} depth={} bit={} faces {}->{}",
                  in.diameter_mm, in.depth_mm, in.bit_type,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometrically identical to drill_hole — same cylindrical-face +
// circular-edge pattern.  We replay metadata when present (HIGH confidence)
// and emit a medium-confidence geometric candidate otherwise, which the
// process planner can disambiguate using the workpiece material.

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

    // 1) Metadata replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }

    // 2) Geometric fallback — find drilled bores.
    const TopoDS_Shape& shape = wp.shape();
    const auto edgeFaces = buildEdgeFaceMap(shape);

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const gp_Ax1 axis = cyl.Axis();

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

        // Discriminate blind / through by smallest adjacent planar face area.
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
            { "diameter_mm",   2.0 * radius },
            { "depth_mm",      through ? 0.0 : depth },
            { "through_hole",  through },
            { "bit_type",      "brad_point" },
            { "end_grain",     false },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "source",              "geometry" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.60, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::wood_drilling
