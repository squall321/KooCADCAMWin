// @lat: [[engine/skills#clinch_nut]]

#include "clinch_nut.hpp"

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

namespace koocadcam::skill::clinch_nut {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Clinch-nut barrel-hole sizes (mm) ────────────────────────────────────
//
// Slice-1 simplification: barrel_dia = thread_dia + 0.5 mm.  Real PEM tables
// vary by series (CL / CLS / SL etc.); 0.5 mm is the most common allowance.

namespace {

struct ClinchSpec {
    const char* size;
    double      thread_dia_mm;
    double      barrel_dia_mm;
};

constexpr std::array<ClinchSpec, 5> kClinchTable {{
    { "M3", 3.0, 3.5 },
    { "M4", 4.0, 4.5 },
    { "M5", 5.0, 5.5 },
    { "M6", 6.0, 6.5 },
    { "M8", 8.0, 8.5 },
}};

const ClinchSpec* findSpec(const std::string& thread_size)
{
    for (const auto& e : kClinchTable)
        if (thread_size == e.size) return &e;
    return nullptr;
}

}  // namespace

double clinchBarrelDiameter(const std::string& thread_size)
{
    const ClinchSpec* s = findSpec(thread_size);
    return s ? s->barrel_dia_mm : 0.0;
}

std::string threadSizeForBarrel(double barrel_dia_mm, double tol_mm)
{
    const ClinchSpec* best = nullptr;
    double bestErr = tol_mm;
    for (const auto& e : kClinchTable) {
        const double err = std::abs(e.barrel_dia_mm - barrel_dia_mm);
        if (err < bestErr) { bestErr = err; best = &e; }
    }
    return best ? std::string(best->size) : std::string();
}

double senseSheetThickness(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thread_size.empty()) {
        r.add("DFM-CN-SIZE", "error", "clinch_nut: thread_size is empty");
        return r;
    }
    if (clinchBarrelDiameter(in.thread_size) <= 0.0) {
        r.add("DFM-CN-SIZE", "error",
              "clinch_nut: unknown thread_size '" + in.thread_size +
              "' (supported: M3/M4/M5/M6/M8)");
        return r;
    }

    // Decide the working sheet thickness: input override OR sensed.
    const double sensedT = senseSheetThickness(wp);
    const double t = (in.sheet_thickness_mm > 0.0)
                       ? in.sheet_thickness_mm
                       : sensedT;

    if (t < 0.5 || t > 3.0) {
        r.add("DFM-CN-SHEET", "error",
              "clinch_nut sheet_thickness " + std::to_string(t) +
              " mm outside supported range [0.5, 3.0] mm");
    }

    // If both the input AND a sensed value are present, warn on mismatch.
    if (in.sheet_thickness_mm > 0.0 && sensedT > 0.0 &&
        std::abs(in.sheet_thickness_mm - sensedT) > 0.2)
    {
        r.add("DFM-CN-MISMATCH", "warning",
              "clinch_nut sheet_thickness input " +
              std::to_string(in.sheet_thickness_mm) +
              " mm disagrees with sensed " + std::to_string(sensedT) + " mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "clinch_nut DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("clinch_nut: entry_face datum unresolved");

    const double barrelDia = clinchBarrelDiameter(in.thread_size);
    const double t = (in.sheet_thickness_mm > 0.0)
                       ? in.sheet_thickness_mm
                       : senseSheetThickness(wp);

    // Through-cut geometry: same approach as sheet_punch / rivet_hole.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

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
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax) / 2.0 - adir.Z() * margin);
    }
    const double toolHeight = bboxDiag + 2.0 * kOverhang;
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, barrelDia / 2.0, toolHeight);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",        *entryId },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size",          in.thread_size },
        { "sheet_thickness_mm",   t },
        { "barrel_dia_mm",        barrelDia },
        { "sheet_thickness_input", in.sheet_thickness_mm },   // 0 = auto
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", false },
        { "thread_size",                in.thread_size },
        { "sheet_thickness_mm",         t },
        { "barrel_dia_mm",              barrelDia },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "punch_press;clinch_nut_install";
    tooling.tool_dia_mm       = barrelDia;
    tooling.tool_length_mm    = t * 2.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = M_PI * (barrelDia / 2.0) * (barrelDia / 2.0) * t;
    tooling.est_cycle_time_s  = 1.0;          // punch + press
    tooling.extra = {
        { "machining_constraint",
          "Hole pierced first, then clinch nut pressed in; sheet plastically "
          "deforms into knurl/serrations to retain the nut.  Deformation not "
          "modelled in slice-1." },
        { "fastener_class",   "clinch_nut" },
        { "thread_size",      in.thread_size },
        { "sheet_thickness_mm", t },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::clinch_nut applied: {} sheet={} barrel={} faces {}→{}",
                  in.thread_size, t, barrelDia,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay → topology fallback.  Topology fallback restricts to thin
// stock (sheet_thickness ≤ 3 mm) and requires the cylinder dia to match a
// clinch-nut barrel size.

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

    // History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",      f.params.value("position_x_mm",      0.0) },
            { "position_y_mm",      f.params.value("position_y_mm",      0.0) },
            { "axis_dir",           f.params.value("axis_dir",           json::array({0.0,0.0,-1.0})) },
            { "thread_size",        f.params.value("thread_size",        std::string()) },
            { "sheet_thickness_mm", f.params.value("sheet_thickness_mm", 0.0) },
            { "barrel_dia_mm",      f.params.value("barrel_dia_mm",      0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology fallback: only fires on sheet stock (≤ 3.0 mm).
    const double t = senseSheetThickness(wp);
    if (t <= 0.0 || t > 3.0) return out;

    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const double dia    = 2.0 * radius;

        const std::string size = threadSizeForBarrel(dia, 0.10);
        if (size.empty()) continue;

        // Verify the cylinder spans the sheet (through-hole).
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
        const gp_Pnt cBot = *minIt;
        const gp_Pnt cTop = *maxIt;
        const double axialExtent = cTop.Distance(cBot);

        // Through-hole in this sheet?
        if (std::abs(axialExtent - t) > std::max(0.05, t * 0.05)) continue;

        // Both bounding circles must touch large planar faces (sheet faces).
        const double drillBottomArea = M_PI * radius * radius;
        double minAdjPlanarArea = std::numeric_limits<double>::max();
        int planarAdjCount = 0;
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
                ++planarAdjCount;
            }
        }
        if (planarAdjCount < 2) continue;
        if (minAdjPlanarArea < drillBottomArea * 1.5) continue;

        gp_Vec dirVec(cTop, cBot);
        if (dirVec.Magnitude() < 1e-9) continue;
        dirVec.Normalize();

        json recovered = {
            { "position_x_mm",      cTop.X() },
            { "position_y_mm",      cTop.Y() },
            { "axis_dir",           { dirVec.X(), dirVec.Y(), dirVec.Z() } },
            { "thread_size",        size },
            { "sheet_thickness_mm", t },
            { "barrel_dia_mm",      dia },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center", { cTop.X(), cTop.Y(), cTop.Z() } },
            { "bot_center", { cBot.X(), cBot.Y(), cBot.Z() } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.80, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::clinch_nut
