// @lat: [[engine/skills#bolted_joint]]

#include "bolted_joint.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
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
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace koocadcam::skill::bolted_joint {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Standard bolt thread nominal diameters (ISO metric coarse) ───────────
//
// Sourced from the central thread table (kMetricThreads) for M3+.  M1.6,
// M2, and M2.5 are kept as a local fallback because central skips them.
namespace {

struct ThreadEntry { const char* code; double nominal_mm; };
constexpr std::array<ThreadEntry, 3> kLocalSmallThreads = {{
    { "M1.6", 1.6 },
    { "M2",   2.0 },
    { "M2.5", 2.5 },
}};

}  // namespace

double boltNominalDia(const std::string& bolt_thread)
{
    if (const auto* m = thread_table::findMetric(bolt_thread)) {
        return m->nominal_dia_mm;
    }
    for (const auto& t : kLocalSmallThreads) {
        if (bolt_thread == t.code) return t.nominal_mm;
    }
    return 0.0;
}

double holeDiameterFor(const std::string& bolt_thread, double clearance_mm)
{
    const double nom = boltNominalDia(bolt_thread);
    if (nom <= 0.0) return 0.0;
    return nom + clearance_mm;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const double nom = boltNominalDia(in.bolt_thread);
    if (nom <= 0.0) {
        r.add("DFM-BOLT-DIA", "error",
              "bolted_joint bolt_thread '" + in.bolt_thread +
              "' not in known table (M1.6..M16)");
    } else if (nom < 1.6) {
        r.add("DFM-BOLT-DIA", "error",
              "bolted_joint bolt nominal " + std::to_string(nom) +
              " mm < 1.6 mm minimum");
    }
    if (in.clearance_mm < 0.05 || in.clearance_mm > 1.0) {
        r.add("DFM-BOLT-FIT", "error",
              "bolted_joint clearance_mm " + std::to_string(in.clearance_mm) +
              " outside [0.05, 1.0] mm");
    }
    if (in.torque_Nm <= 0.0) {
        r.add("DFM-BOLT-TORQUE", "error",
              "bolted_joint torque_Nm must be > 0");
    }
    if (in.preload_N <= 0.0) {
        r.add("DFM-BOLT-PRELD", "error",
              "bolted_joint preload_N must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bolted_joint DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("bolted_joint: entry_face datum unresolved");

    const double holeDia = holeDiameterFor(in.bolt_thread, in.clearance_mm);
    if (holeDia <= 0.0) {
        throw SkillError("bolted_joint: hole diameter computed as 0");
    }

    // Geometry — through-hole identical pattern to drill_hole(through_hole=true).
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
            in.position_x_mm - adir.X() * (margin + kEntryOverhang),
            in.position_y_mm - adir.Y() * (margin + kEntryOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (margin + kEntryOverhang));
    }

    const double toolHeight = bboxDiag + 2.0 * kEntryOverhang;
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, holeDia / 2.0, toolHeight);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "bolt_thread",    in.bolt_thread },
        { "clearance_mm",   in.clearance_mm },
        { "hole_dia_mm",    holeDia },
        { "torque_Nm",      in.torque_Nm },
        { "preload_N",      in.preload_N },
        { "through_hole",   true },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", false },
        { "bolt_thread",                in.bolt_thread },
        { "hole_dia_mm",                holeDia },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "additive",                   false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = holeDia;
    tooling.tool_length_mm    = bboxDiag * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = M_PI * (holeDia / 2.0) * (holeDia / 2.0) * bboxDiag;
    tooling.est_cycle_time_s  = std::max(1.0, bboxDiag / 50.0);
    tooling.extra = {
        { "fastener_type",  "bolt" },
        { "bolt_thread",    in.bolt_thread },
        { "torque_Nm",      in.torque_Nm },
        { "preload_N",      in.preload_N },
        { "assembly_step",  "torque_to_spec_then_torque_check" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bolted_joint applied: thread={} hole={} torque={} faces {}→{}",
                  in.bolt_thread, holeDia, in.torque_Nm,
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

// Match a hole diameter to the closest bolt thread that gives a clearance in
// [0.05, 1.0] mm.  Returns the matching thread code or "".
std::string matchBoltThread(double hole_dia_mm, double& matched_clearance_mm)
{
    matched_clearance_mm = 0.0;
    std::string best;
    double bestDiff = std::numeric_limits<double>::max();
    auto tryEntry = [&](const char* code, double nom) {
        const double clearance = hole_dia_mm - nom;
        if (clearance < 0.04 || clearance > 1.05) return;
        if (clearance < bestDiff) {
            bestDiff = clearance;
            best = code;
            matched_clearance_mm = clearance;
        }
    };
    for (const auto& m : thread_table::kMetricThreads) {
        // Only consider sizes up to M16 — original table cap.
        if (m.nominal_dia_mm > 16.0 + 1e-6) continue;
        tryEntry(m.size_key, m.nominal_dia_mm);
    }
    for (const auto& t : kLocalSmallThreads) {
        tryEntry(t.code, t.nominal_mm);
    }
    return best;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // 1) History replay — high confidence (1.0).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm", f.params.value("position_x_mm", 0.0) },
            { "position_y_mm", f.params.value("position_y_mm", 0.0) },
            { "axis_dir",      f.params.value("axis_dir",      json::array({0.0,0.0,-1.0})) },
            { "bolt_thread",   f.params.value("bolt_thread",   std::string("M5")) },
            { "clearance_mm",  f.params.value("clearance_mm",  0.5) },
            { "hole_dia_mm",   f.params.value("hole_dia_mm",   0.0) },
            { "torque_Nm",     f.params.value("torque_Nm",     0.0) },
            { "preload_N",     f.params.value("preload_N",     0.0) },
            { "through_hole",  true },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // 2) Geometric fallback.
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const double dia    = 2.0 * radius;

        double clearance = 0.0;
        const std::string thread = matchBoltThread(dia, clearance);
        if (thread.empty()) continue;

        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();

        // Collect the two bounding circles.
        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3) continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;

        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - axis.Location().X()) * adir.X() +
                   (p.Y() - axis.Location().Y()) * adir.Y() +
                   (p.Z() - axis.Location().Z()) * adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) {
            return projOnAxis(a) < projOnAxis(b);
        };
        const auto minIt = std::min_element(centers.begin(), centers.end(), cmp);
        const auto maxIt = std::max_element(centers.begin(), centers.end(), cmp);
        const gp_Pnt centerLow  = *minIt;
        const gp_Pnt centerHigh = *maxIt;

        // Must be through.
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
        if (!through) continue;

        gp_Vec drillVec(centerHigh, centerLow);
        if (drillVec.Magnitude() < 1e-9) continue;
        drillVec.Normalize();

        json recovered = {
            { "position_x_mm", centerHigh.X() },
            { "position_y_mm", centerHigh.Y() },
            { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "bolt_thread",   thread },
            { "clearance_mm",  clearance },
            { "hole_dia_mm",   dia },
            { "torque_Nm",     0.0 },     // unknown without metadata
            { "preload_N",     0.0 },     // unknown without metadata
            { "through_hole",  true },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "matched_clearance_mm", clearance },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::bolted_joint
