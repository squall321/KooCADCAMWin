// @lat: [[engine/skills#rivet_hole]]

#include "rivet_hole.hpp"

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
#include <array>
#include <cmath>
#include <limits>

namespace koocadcam::skill::rivet_hole {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Standard rivet diameters ─────────────────────────────────────────────

const std::vector<double>& standardRivetDiameters()
{
    // Common metric + imperial blind-rivet body diameters.
    static const std::vector<double> kStd = {
        1.6, 2.0, 2.4, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0
    };
    return kStd;
}

namespace {

// Imperial fractional inch → mm mapping for the standard set.
struct ImperialMap { const char* code; double mm; };
constexpr std::array<ImperialMap, 6> kImperialMap = {{
    { "ANSI_1_16in", 1.5875 },
    { "ANSI_3_32in", 2.38125 },
    { "ANSI_1_8in",  3.175 },
    { "ANSI_5_32in", 3.96875 },
    { "ANSI_3_16in", 4.7625 },
    { "ANSI_1_4in",  6.35 },
}};

// Metric M-class (rivet body nominal = M code numeral, mm).
constexpr std::array<const char*, 9> kMetricCodes = {
    "M1.6", "M2", "M2.4", "M3", "M4", "M5", "M6", "M8", "M10"
};

}  // namespace

std::string classFromDiameter(double dia_mm)
{
    if (dia_mm <= 0.0) return {};
    // Try metric first — closest standard within 0.15 mm.
    const auto& kStd = standardRivetDiameters();
    double bestDiff = std::numeric_limits<double>::max();
    int    bestIdx  = -1;
    for (int i = 0; i < static_cast<int>(kStd.size()); ++i) {
        const double d = std::abs(kStd[i] - dia_mm);
        if (d < bestDiff) { bestDiff = d; bestIdx = i; }
    }
    if (bestIdx >= 0 && bestDiff <= 0.15) {
        return kMetricCodes[bestIdx];
    }
    // Otherwise check imperial.
    bestDiff = std::numeric_limits<double>::max();
    int impIdx = -1;
    for (size_t i = 0; i < kImperialMap.size(); ++i) {
        const double d = std::abs(kImperialMap[i].mm - dia_mm);
        if (d < bestDiff) { bestDiff = d; impIdx = static_cast<int>(i); }
    }
    if (impIdx >= 0 && bestDiff <= 0.15) {
        return kImperialMap[impIdx].code;
    }
    return {};
}

double diameterFromClass(const std::string& cls)
{
    const auto& kStd = standardRivetDiameters();
    for (size_t i = 0; i < kMetricCodes.size(); ++i) {
        if (cls == kMetricCodes[i]) return kStd[i];
    }
    for (const auto& im : kImperialMap) {
        if (cls == im.code) return im.mm;
    }
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "rivet_hole diameter must be > 0");
    }
    if (in.diameter_mm > 0.0 && in.diameter_mm < 2.0) {
        r.add("DFM-RIVET-MIN", "error",
              "rivet_hole diameter " + std::to_string(in.diameter_mm) +
              " mm < min 2.0 mm (smallest standard blind rivet)");
    }
    // If a class is given, sanity-check it matches the diameter.
    if (!in.rivet_diameter_class.empty()) {
        const double classDia = diameterFromClass(in.rivet_diameter_class);
        if (classDia > 0.0 && std::abs(classDia - in.diameter_mm) > 0.5) {
            r.add("DFM-RIVET-CLASS", "warning",
                  "rivet_hole class " + in.rivet_diameter_class +
                  " (= " + std::to_string(classDia) + " mm) does not match "
                  "diameter " + std::to_string(in.diameter_mm) + " mm");
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
        std::string msg = "rivet_hole DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("rivet_hole: entry_face datum unresolved");

    // Geometry follows drill_hole(through_hole=true).
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
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Resolve class string — use input if non-empty, otherwise derive.
    const std::string rivetClass = in.rivet_diameter_class.empty()
        ? classFromDiameter(in.diameter_mm)
        : in.rivet_diameter_class;

    json params = {
        { "entry_face_id",         *entryId },
        { "position_x_mm",         in.position_x_mm },
        { "position_y_mm",         in.position_y_mm },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",           in.diameter_mm },
        { "through_hole",          true },
        { "rivet_diameter_class",  rivetClass },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", false },
        { "diameter_mm",                in.diameter_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "rivet_diameter_class",       rivetClass },
        { "through_hole",               true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "rivet_drill";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = bboxDiag * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;       // tighter than general drill (precision hole)
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) *
                                (in.diameter_mm / 2.0) * bboxDiag;
    tooling.est_cycle_time_s  = std::max(1.0, bboxDiag / 60.0);
    tooling.extra = {
        { "tolerance_class",      "H10" },  // typical rivet hole tolerance
        { "rivet_diameter_class", rivetClass },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rivet_hole applied: dia={} class={} faces {}→{}",
                  in.diameter_mm, rivetClass,
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
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const double dia    = 2.0 * radius;
        if (dia < 1.4 || dia > 11.0) continue;   // outside rivet range

        // Standard rivet match — recognize only if dia is within 0.15 of a
        // standard size.
        const std::string cls = classFromDiameter(dia);
        if (cls.empty()) continue;

        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();

        // Collect bounding circles.
        std::vector<gp_Pnt> circleCenters;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3)
                continue;
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

        // Must be THROUGH — same area test as drill_hole.
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

        // Confidence high if exact standard, lower for approximate.
        const double stdDia = diameterFromClass(cls);
        const double err = std::abs(stdDia - dia);
        double conf = 0.90;
        if (err > 0.05) conf -= 0.20;
        if (err > 0.10) conf -= 0.20;
        conf = std::clamp(conf, 0.30, 0.95);

        json recovered = {
            { "position_x_mm",         centerHigh.X() },
            { "position_y_mm",         centerHigh.Y() },
            { "axis_dir",              { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",           dia },
            { "through_hole",          true },
            { "rivet_diameter_class",  cls },
        };
        json matched = {
            { "cylindrical_face_id",   fIdx },
            { "top_center",            { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bot_center",            { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "standard_match_err_mm", err },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::rivet_hole
