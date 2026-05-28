// @lat: [[engine/skills#press_assembly]]

#include "press_assembly.hpp"

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

namespace koocadcam::skill::press_assembly {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// Standard pin sizes — same table as press_fit (kept private to avoid header
// coupling between assembly-level and part-level skills).
namespace {

const std::vector<double>& standardPinDiameters()
{
    static const std::vector<double> kStd = {
        1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0,
        12.0, 16.0, 20.0, 25.0, 32.0, 40.0, 50.0
    };
    return kStd;
}

}  // namespace

double holeDiameterFor(double pin_diameter_mm, double interference_um)
{
    return pin_diameter_mm + interference_um * 0.001;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pin_diameter_mm < 1.5 || in.pin_diameter_mm > 50.0) {
        r.add("DFM-PA-PIN", "error",
              "press_assembly pin_diameter_mm " + std::to_string(in.pin_diameter_mm) +
              " outside [1.5, 50] mm");
    }
    if (in.interference_um > -5.0) {
        r.add("DFM-PA-INT", "error",
              "press_assembly interference_um " + std::to_string(in.interference_um) +
              " > -5 µm — not an interference fit (need ≤ -5 µm)");
    } else if (in.interference_um < -100.0) {
        r.add("DFM-PA-INT", "error",
              "press_assembly interference_um " + std::to_string(in.interference_um) +
              " < -100 µm — excessive interference, may shear pin");
    }
    if (in.press_force_N <= 0.0) {
        r.add("DFM-PA-FORCE", "error",
              "press_assembly press_force_N must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "press_assembly DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("press_assembly: entry_face datum unresolved");

    const double holeDia = holeDiameterFor(in.pin_diameter_mm, in.interference_um);
    if (holeDia <= 0.0) {
        throw SkillError("press_assembly: computed hole diameter " +
                         std::to_string(holeDia) + " mm ≤ 0");
    }

    // Always through — geometry follows the drill_hole(through_hole=true) pattern.
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
        { "entry_face_id",     *entryId },
        { "position_x_mm",     in.position_x_mm },
        { "position_y_mm",     in.position_y_mm },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
        { "pin_diameter_mm",   in.pin_diameter_mm },
        { "interference_um",   in.interference_um },
        { "hole_dia_mm",       holeDia },
        { "press_force_N",     in.press_force_N },
        { "partner_part",      in.partner_part },
        { "through_hole",      true },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", false },
        { "pin_diameter_mm",            in.pin_diameter_mm },
        { "interference_um",            in.interference_um },
        { "hole_dia_mm",                holeDia },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "additive",                   false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "reamer";
    tooling.tool_dia_mm       = holeDia;
    tooling.tool_length_mm    = bboxDiag * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 6;
    tooling.cutting_speed_sfm = 120.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = M_PI * (holeDia / 2.0) * (holeDia / 2.0) * bboxDiag;
    tooling.est_cycle_time_s  = std::max(1.5, bboxDiag / 40.0);
    tooling.extra = {
        { "tolerance_class",   "H7" },
        { "fit_class",         "press_assembly" },
        { "pin_diameter_mm",   in.pin_diameter_mm },
        { "interference_um",   in.interference_um },
        { "press_force_N",     in.press_force_N },
        { "partner_part",      in.partner_part },
        { "assembly_step",     "align_pin_then_press_to_seat_depth" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::press_assembly applied: pin={} int={}µm force={} faces {}→{}",
                  in.pin_diameter_mm, in.interference_um, in.press_force_N,
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

struct PinMatch { double nominal_dia = 0.0; double interference_um = 0.0; };

PinMatch matchStandardPin(double hole_dia_mm)
{
    PinMatch best;
    double bestErr = std::numeric_limits<double>::max();
    for (double std : standardPinDiameters()) {
        const double interfMm = hole_dia_mm - std;
        const double interfUm = interfMm * 1000.0;
        if (interfUm > -5.0 || interfUm < -100.0) continue;
        const double err = std::abs(interfUm + 25.0);
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

    // 1) History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",   f.params.value("position_x_mm",   0.0) },
            { "position_y_mm",   f.params.value("position_y_mm",   0.0) },
            { "axis_dir",        f.params.value("axis_dir",        json::array({0.0,0.0,-1.0})) },
            { "pin_diameter_mm", f.params.value("pin_diameter_mm", 0.0) },
            { "interference_um", f.params.value("interference_um", 0.0) },
            { "hole_dia_mm",     f.params.value("hole_dia_mm",     0.0) },
            { "press_force_N",   f.params.value("press_force_N",   0.0) },
            { "partner_part",    f.params.value("partner_part",    std::string()) },
            { "through_hole",    true },
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

        const PinMatch pm = matchStandardPin(dia);
        if (pm.nominal_dia <= 0.0) continue;

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
            { "position_x_mm",   centerHigh.X() },
            { "position_y_mm",   centerHigh.Y() },
            { "axis_dir",        { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "pin_diameter_mm", pm.nominal_dia },
            { "interference_um", pm.interference_um },
            { "hole_dia_mm",     dia },
            { "press_force_N",   0.0 },        // unknown without metadata
            { "partner_part",    std::string() },
            { "through_hole",    true },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center", { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bot_center", { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.50, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::press_assembly
