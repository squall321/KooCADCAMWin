// @lat: [[engine/skills#threaded_insert]]

#include "threaded_insert.hpp"

#include "Workpiece.hpp"
#include "drill_and_tap.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::threaded_insert {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Standard insert sizes ────────────────────────────────────────────────
//
// Supported subset of metric thread sizes for slice-1 (M3 … M8).
// Pilot allowance over the tap-drill chart is 0.2 mm — typical Heli-Coil
// install-hole spec.

namespace {

struct InsertSpec {
    const char* size;
    double      nominal_dia_mm;
    double      tap_drill_dia_mm;   // ISO 68-1 coarse pilot (matches drill_and_tap chart)
};

// drill_and_tap covers M1.6 … M6 only.  For M8 (≥ 6.8 mm pilot in ISO coarse)
// we keep a small local supplement so threaded_insert can stand alone.
constexpr std::array<InsertSpec, 5> kInsertTable {{
    { "M3", 3.0, 2.50 },
    { "M4", 4.0, 3.30 },
    { "M5", 5.0, 4.20 },
    { "M6", 6.0, 5.00 },
    { "M8", 8.0, 6.80 },
}};

constexpr double kInsertPilotAllowance_mm = 0.2;

const InsertSpec* findInsertSpec(const std::string& thread_size)
{
    for (const auto& e : kInsertTable)
        if (thread_size == e.size) return &e;
    return nullptr;
}

}  // namespace

double threadNominalDia(const std::string& thread_size)
{
    const InsertSpec* s = findInsertSpec(thread_size);
    return s ? s->nominal_dia_mm : 0.0;
}

double insertPilotDiameter(const std::string& thread_size)
{
    const InsertSpec* s = findInsertSpec(thread_size);
    if (!s) return 0.0;
    // Sanity: cross-check against drill_and_tap chart when sizes overlap.
    const double tapPilot = drill_and_tap::pilotDiameterFor(thread_size);
    const double base = (tapPilot > 0.0) ? tapPilot : s->tap_drill_dia_mm;
    return base + kInsertPilotAllowance_mm;
}

std::string threadSizeForInsertPilot(double pilot_dia_mm, double tol_mm)
{
    const InsertSpec* best = nullptr;
    double bestErr = tol_mm;
    for (const auto& e : kInsertTable) {
        const double pilot = insertPilotDiameter(e.size);
        if (pilot <= 0.0) continue;
        const double err = std::abs(pilot - pilot_dia_mm);
        if (err < bestErr) { bestErr = err; best = &e; }
    }
    return best ? std::string(best->size) : std::string();
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thread_size.empty()) {
        r.add("DFM-TI-SIZE", "error", "threaded_insert: thread_size is empty");
        return r;
    }
    const double nominal = threadNominalDia(in.thread_size);
    if (nominal <= 0.0) {
        r.add("DFM-TI-SIZE", "error",
              "threaded_insert: unknown thread_size '" + in.thread_size +
              "' (supported: M3/M4/M5/M6/M8)");
        return r;
    }
    if (in.insert_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "threaded_insert: insert_length_mm must be > 0");
    }
    // Minimum engagement: 1.5 × thread diameter.
    if (in.insert_length_mm > 0.0 && in.insert_length_mm < 1.5 * nominal) {
        r.add("DFM-TI-LEN", "error",
              "threaded_insert insert_length " + std::to_string(in.insert_length_mm) +
              " mm < 1.5 × thread dia (" + std::to_string(1.5 * nominal) +
              " mm) — insufficient engagement");
    }
    if (in.insert_material.empty()) {
        r.add("DFM-INPUT", "error",
              "threaded_insert: insert_material must be non-empty");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "threaded_insert DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("threaded_insert: entry_face datum unresolved");

    const double pilotDia = insertPilotDiameter(in.thread_size);
    if (pilotDia <= 0.0) {
        throw SkillError("threaded_insert: cannot resolve pilot diameter for " +
                         in.thread_size);
    }

    // Geometry mirrors drill_hole.  The hole is deep enough to fully house
    // the insert plus a small extra (1 mm) for chip / coil clearance.
    constexpr double kExtraClearance_mm = 1.0;
    const double drillDepth = in.insert_length_mm + kExtraClearance_mm;

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

    const double toolHeight = drillDepth + kEntryOverhang;
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, pilotDia / 2.0, toolHeight);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",     *entryId },
        { "position_x_mm",     in.position_x_mm },
        { "position_y_mm",     in.position_y_mm },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size",       in.thread_size },
        { "insert_length_mm",  in.insert_length_mm },
        { "insert_material",   in.insert_material },
        { "pilot_dia_mm",      pilotDia },
        { "drill_depth_mm",    drillDepth },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", true },
        { "thread_size",                in.thread_size },
        { "insert_length_mm",           in.insert_length_mm },
        { "insert_material",            in.insert_material },
        { "pilot_dia_mm",               pilotDia },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;insert_install";
    tooling.tool_dia_mm       = pilotDia;
    tooling.tool_length_mm    = drillDepth * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = M_PI * (pilotDia / 2.0) * (pilotDia / 2.0) * drillDepth;
    tooling.est_cycle_time_s  = std::max(1.0, drillDepth / 50.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type",   "drill" },
              { "tool_dia_mm", pilotDia },
              { "depth_mm",    drillDepth } },
            { { "tool_type",   "insert_install" },
              { "thread_size", in.thread_size },
              { "insert_length_mm", in.insert_length_mm },
              { "insert_material",  in.insert_material } },
        } },
        { "note",
          "Helical coil or press-in insert install not modelled in slice-1;"
          " geometry is the install hole only; install op carried as metadata." }
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::threaded_insert applied: {} (pilot {} × {})",
                  in.thread_size, pilotDia, drillDepth);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay (lossless), then topology fallback that matches the
// install pilot diameter against the insert table.  No through/blind
// disambiguation is done here — a threaded_insert can be either, and the
// caller can inspect the bounding circles if needed.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay first.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",    f.params.value("position_x_mm",    0.0) },
            { "position_y_mm",    f.params.value("position_y_mm",    0.0) },
            { "axis_dir",         f.params.value("axis_dir",         json::array({0.0,0.0,-1.0})) },
            { "thread_size",      f.params.value("thread_size",      std::string()) },
            { "insert_length_mm", f.params.value("insert_length_mm", 0.0) },
            { "insert_material",  f.params.value("insert_material",  std::string()) },
            { "pilot_dia_mm",     f.params.value("pilot_dia_mm",     0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology fallback.

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const double dia    = 2.0 * radius;

        const std::string size = threadSizeForInsertPilot(dia, 0.10);
        if (size.empty()) continue;

        // Recover position & depth.
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
        const gp_Pnt centerHigh = *maxIt;
        const gp_Pnt centerLow  = *minIt;
        const double depth = centerHigh.Distance(centerLow);

        // Assume insert_length ≈ depth - 1 mm clearance (synthesis convention).
        const double insertLenGuess = std::max(0.0, depth - 1.0);

        gp_Vec dirVec(centerHigh, centerLow);
        if (dirVec.Magnitude() < 1e-9) continue;
        dirVec.Normalize();

        json recovered = {
            { "position_x_mm",    centerHigh.X() },
            { "position_y_mm",    centerHigh.Y() },
            { "axis_dir",         { dirVec.X(), dirVec.Y(), dirVec.Z() } },
            { "thread_size",      size },
            { "insert_length_mm", insertLenGuess },
            { "insert_material",  std::string("unknown") },
            { "pilot_dia_mm",     dia },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center", { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "bot_center", { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
        };
        // Confidence modest — topology overlaps with a plain drill_hole.
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::threaded_insert
