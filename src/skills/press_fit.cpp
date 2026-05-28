// @lat: [[engine/skills#press_fit]]

#include "press_fit.hpp"

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
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace koocadcam::skill::press_fit {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Standard press-fit pin sizes (mm) ─────────────────────────────────────
//
// Commonly-stocked dowel-pin diameters used in metal-front consumer assembly
// & jig-fixturing.  Recognize() requires the hole to be near one of these
// (within typical interference range) to claim press_fit confidently.
const std::vector<double>& standardPinDiameters()
{
    static const std::vector<double> kStd = {
        1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0,
        12.0, 16.0, 20.0, 25.0, 32.0, 40.0, 50.0
    };
    return kStd;
}

double holeDiameterFor(double nominal_pin_dia_mm, double interference_um)
{
    return nominal_pin_dia_mm + interference_um * 0.001;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.nominal_pin_dia_mm < 1.5 || in.nominal_pin_dia_mm > 50.0) {
        r.add("DFM-PF-PIN", "error",
              "press_fit nominal_pin_dia " + std::to_string(in.nominal_pin_dia_mm) +
              " mm outside supported range [1.5, 50] mm");
    }
    // Interference is signed; "more negative" = tighter.  Reject zero / positive
    // (zero = slip fit; positive = loose) and absurd under-sizing.
    if (in.interference_um > -5.0) {
        r.add("DFM-PF-INT", "error",
              "press_fit interference_um " + std::to_string(in.interference_um) +
              " > -5 µm — not an interference fit (need ≤ -5 µm on diameter)");
    } else if (in.interference_um < -100.0) {
        r.add("DFM-PF-INT", "error",
              "press_fit interference_um " + std::to_string(in.interference_um) +
              " < -100 µm — excessive interference, may shear pin");
    }
    if (!in.through_hole && in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "press_fit depth_mm must be > 0 for blind hole");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "press_fit DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("press_fit: entry_face datum unresolved");

    const double holeDia = holeDiameterFor(in.nominal_pin_dia_mm, in.interference_um);
    if (holeDia <= 0.0) {
        throw SkillError("press_fit: computed hole diameter " +
                         std::to_string(holeDia) + " mm ≤ 0");
    }

    // Geometry mirrors drill_hole.
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

    const double toolHeight = in.through_hole
        ? (bboxDiag + 2.0 * kEntryOverhang)
        : (in.depth_mm + kEntryOverhang);

    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, holeDia / 2.0, toolHeight);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",      *entryId },
        { "position_x_mm",      in.position_x_mm },
        { "position_y_mm",      in.position_y_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "nominal_pin_dia_mm", in.nominal_pin_dia_mm },
        { "interference_um",    in.interference_um },
        { "hole_dia_mm",        holeDia },
        { "depth_mm",           in.depth_mm },
        { "through_hole",       in.through_hole },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", !in.through_hole },
        { "nominal_pin_dia_mm",         in.nominal_pin_dia_mm },
        { "interference_um",            in.interference_um },
        { "hole_dia_mm",                holeDia },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "reamer";
    tooling.tool_dia_mm       = holeDia;
    tooling.tool_length_mm    = (in.through_hole ? bboxDiag : in.depth_mm) * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 6;
    tooling.cutting_speed_sfm = 120.0;        // reaming is slower than drilling
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = M_PI * (holeDia / 2.0) * (holeDia / 2.0) *
        (in.through_hole ? bboxDiag : in.depth_mm);
    tooling.est_cycle_time_s  = std::max(1.5,
        (in.through_hole ? bboxDiag : in.depth_mm) / 40.0);
    tooling.extra = {
        { "tolerance_class",       "H7" },
        { "fit_class",             "press" },
        { "nominal_pin_dia_mm",    in.nominal_pin_dia_mm },
        { "interference_um",       in.interference_um },
        { "machining_constraint",
          "tight diameter — pilot drill then ream; check pin straightness before press." }
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::press_fit applied: pin={} int={}µm hole={} faces {}→{}",
                  in.nominal_pin_dia_mm, in.interference_um, holeDia,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay first (lossless), then topology fallback.  Topology
// fallback only fires when the bore diameter matches "standard_pin - {5..100}µm"
// for one of the listed standard pin sizes, because a plain drill at the
// same diameter is geometrically indistinguishable.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

// Find the closest standard pin size and its implied interference.
// Returns {nominal_dia, interference_um}.  interference is 0 if no match.
struct PinMatch { double nominal_dia = 0.0; double interference_um = 0.0; };

PinMatch matchStandardPin(double hole_dia_mm)
{
    PinMatch best;
    double bestErr = std::numeric_limits<double>::max();
    for (double std : standardPinDiameters()) {
        // The hole is undersized → standard_pin > hole_dia.
        // Try this pin only if it gives a tight (5..100 µm) interference.
        const double interfMm = hole_dia_mm - std;          // negative if undersized
        const double interfUm = interfMm * 1000.0;
        if (interfUm > -5.0 || interfUm < -100.0) continue;  // outside press-fit range
        const double err = std::abs(interfUm + 25.0);        // prefer near nominal -25 µm
        if (err < bestErr) {
            bestErr = err;
            best.nominal_dia = std;
            best.interference_um = interfUm;
        }
    }
    return best;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay (high confidence).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",      f.params.value("position_x_mm",      0.0) },
            { "position_y_mm",      f.params.value("position_y_mm",      0.0) },
            { "axis_dir",           f.params.value("axis_dir",           json::array({0.0,0.0,-1.0})) },
            { "nominal_pin_dia_mm", f.params.value("nominal_pin_dia_mm", 0.0) },
            { "interference_um",    f.params.value("interference_um",    0.0) },
            { "hole_dia_mm",        f.params.value("hole_dia_mm",        0.0) },
            { "depth_mm",           f.params.value("depth_mm",           0.0) },
            { "through_hole",       f.params.value("through_hole",       false) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology fallback — search cylindrical faces whose diameter matches a
    // (standard_pin − [5..100] µm) under-size.
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const double dia    = 2.0 * radius;

        const PinMatch pm = matchStandardPin(dia);
        if (pm.nominal_dia <= 0.0) continue;

        // Collect bounding circles to recover position & depth.
        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();
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
        const double depth = centerHigh.Distance(centerLow);

        // Through-or-blind heuristic (same as drill_hole).
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

        gp_Vec drillVec(centerHigh, centerLow);
        if (drillVec.Magnitude() < 1e-9) continue;
        drillVec.Normalize();

        json recovered = {
            { "position_x_mm",      centerHigh.X() },
            { "position_y_mm",      centerHigh.Y() },
            { "axis_dir",           { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "nominal_pin_dia_mm", pm.nominal_dia },
            { "interference_um",    pm.interference_um },
            { "hole_dia_mm",        dia },
            { "depth_mm",           through ? 0.0 : depth },
            { "through_hole",       through },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center", { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bot_center", { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
        };
        // Confidence is modest — plain drill_hole at the same diameter has
        // an identical B-rep; CAPP / user must disambiguate from metadata.
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::press_fit
