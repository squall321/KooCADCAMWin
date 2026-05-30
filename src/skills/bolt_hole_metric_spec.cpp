// @lat: [[engine/skills#bolt_hole_metric_spec]]

#include "bolt_hole_metric_spec.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::bolt_hole_metric_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── ISO 273 clearance hole table ────────────────────────────────────────
//
// Embedded as static-const arrays.  Each row is one metric size with three
// fit-class clearances (close / medium / free).  Source: ISO 273:1979,
// "Fasteners — Clearance holes for bolts and screws".  11 sizes covers
// the consumer-electronics + structural ranges asked for in the brief.

namespace {

struct MetricClearance
{
    const char* size;
    double      close_mm;
    double      medium_mm;
    double      free_mm;
};

constexpr std::array<MetricClearance, 11> kIso273Table {{
    { "M3",   3.2,  3.4,  3.6 },
    { "M4",   4.3,  4.5,  4.8 },
    { "M5",   5.3,  5.5,  5.8 },
    { "M6",   6.4,  6.6,  7.0 },
    { "M8",   8.4,  9.0, 10.0 },
    { "M10", 10.5, 11.0, 12.0 },
    { "M12", 13.0, 14.0, 15.0 },
    { "M16", 17.0, 18.0, 19.0 },
    { "M20", 21.0, 22.0, 24.0 },
    { "M24", 25.0, 26.0, 28.0 },
    { "M30", 31.0, 33.0, 35.0 },
}};

}  // namespace

double clearanceDiameterFor(const std::string& thread_size,
                            const std::string& fit_class)
{
    for (const auto& e : kIso273Table) {
        if (thread_size != e.size) continue;
        if (fit_class == "close")  return e.close_mm;
        if (fit_class == "medium") return e.medium_mm;
        if (fit_class == "free")   return e.free_mm;
        return 0.0;
    }
    return 0.0;
}

// Reverse lookup: given a measured diameter, find the closest (size, fit)
// pair within `tol_mm`.  Used by recognize().
namespace {

struct ReverseMatch
{
    std::string  size;
    std::string  fit;
    double       err;
};

ReverseMatch closestSpecMatch(double dia_mm, double tol_mm)
{
    ReverseMatch best { {}, {}, tol_mm };
    for (const auto& e : kIso273Table) {
        const struct { const char* fit; double d; } cands[3] = {
            { "close",  e.close_mm  },
            { "medium", e.medium_mm },
            { "free",   e.free_mm   },
        };
        for (const auto& c : cands) {
            const double err = std::abs(c.d - dia_mm);
            if (err < best.err) {
                best.err  = err;
                best.size = e.size;
                best.fit  = c.fit;
            }
        }
    }
    return best;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thread_size.empty()) {
        r.add("DFM-METRIC-UNKNOWN", "error",
              "bolt_hole_metric_spec: thread_size is empty");
        return r;
    }
    if (in.fit_class != "close" && in.fit_class != "medium" && in.fit_class != "free") {
        r.add("DFM-METRIC-FIT", "error",
              "bolt_hole_metric_spec: fit_class '" + in.fit_class +
              "' not in {close, medium, free} per ISO 273");
        return r;
    }
    const double clrDia = clearanceDiameterFor(in.thread_size, in.fit_class);
    if (clrDia <= 0.0) {
        r.add("DFM-METRIC-UNKNOWN", "error",
              "bolt_hole_metric_spec: unknown thread_size '" + in.thread_size +
              "' (supported per ISO 273: M3..M30, 11 sizes)");
        return r;
    }

    if (in.chamfer_size_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "bolt_hole_metric_spec: chamfer_size_mm must be >= 0");
    }
    if (in.chamfer_angle_deg < 15.0 || in.chamfer_angle_deg > 75.0) {
        r.add("DFM-011", "error",
              "bolt_hole_metric_spec: chamfer angle " +
              std::to_string(in.chamfer_angle_deg) +
              " outside [15°, 75°] — approaches knife edge");
    }

    // Stock thickness vs clearance dia — VDI 2230 bearing-area heuristic.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thicknessAlongAxis =
        std::abs(in.axis_dir.X()) * (xMax - xMin) +
        std::abs(in.axis_dir.Y()) * (yMax - yMin) +
        std::abs(in.axis_dir.Z()) * (zMax - zMin);
    if (thicknessAlongAxis > 0.0 && thicknessAlongAxis < clrDia) {
        r.add("DFM-METRIC-DEPTH", "warning",
              "bolt_hole_metric_spec: stock_thickness " +
              std::to_string(thicknessAlongAxis) + " mm < clearance_dia " +
              std::to_string(clrDia) +
              " — bearing area below VDI 2230 minimum");
    }
    return r;
}

// ── Synthesis: clearance through-hole + entry chamfer (2-cut compound) ──

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bolt_hole_metric_spec DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double clrDia = clearanceDiameterFor(in.thread_size, in.fit_class);
    const double clrR   = clrDia / 2.0;

    // Bounding info → through-hole length.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const gp_Dir adir = in.axis_dir;
    const double kOverhang = 0.05;

    // Common-case top-face-down drilling
    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0)
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        else
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
    } else {
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    }

    const gp_Ax2 toolAx(toolStart, adir);

    // CUT 1: full-through clearance cylinder (length = bbox diag + 2·overhang).
    const double throughLen = bboxDiag + 2.0 * kOverhang;
    const TopoDS_Shape throughTool = pr::cylinder(toolAx, clrR, throughLen);

    // CUT 2: entry chamfer — a cone frustum.  Big radius = clrR + leg,
    //        small radius = clrR.  Height = leg / tan(angle).
    const double leg = std::max(in.chamfer_size_mm, 0.0);
    const double chamferDepth = (leg > 0.0)
        ? leg / std::tan(in.chamfer_angle_deg * M_PI / 180.0)
        : 0.0;
    const double chamferBigR  = clrR + leg;

    TopoDS_Shape chamferTool;
    if (leg > 1e-6 && chamferDepth > 1e-6) {
        // Cone goes from BigR at entry → clrR at chamferDepth.
        chamferTool = pr::coneFrustum(toolAx, chamferBigR, clrR, chamferDepth);
    }

    // FUSE concentric cuts then single Boolean — same pattern as counterbore.
    TopoDS_Shape fusedCutter = throughTool;
    if (!chamferTool.IsNull()) {
        fusedCutter = pr::fuse(throughTool, chamferTool);
    }
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedCutter);

    // ── Build signature ─────────────────────────────────────────────────
    json params = {
        { "position_x_mm",     in.position_x_mm },
        { "position_y_mm",     in.position_y_mm },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size",       in.thread_size },
        { "fit_class",         in.fit_class },
        { "chamfer_size_mm",   in.chamfer_size_mm },
        { "chamfer_angle_deg", in.chamfer_angle_deg },
        { "clearance_dia_mm",  clrDia },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           2 },
        { "thread_size",                in.thread_size },
        { "fit_class",                  in.fit_class },
        { "clearance_dia_mm",           clrDia },
        { "chamfer_dia_mm",             2.0 * chamferBigR },
        { "chamfer_depth_mm",           chamferDepth },
        { "cylindrical_face_count",     1 },
        { "conical_face_count",         (leg > 0.0) ? 1 : 0 },
        { "circular_edge_count",        (leg > 0.0) ? 3 : 2 },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;chamfer_mill";
    tooling.tool_dia_mm      = clrDia;
    tooling.tool_length_mm   = throughLen + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double cylVol = M_PI * clrR * clrR * throughLen;
    const double conV   = (leg > 0.0)
        ? M_PI * chamferDepth / 3.0 *
          (chamferBigR * chamferBigR + chamferBigR * clrR + clrR * clrR)
        : 0.0;
    const double cylInChamfer = M_PI * clrR * clrR * chamferDepth;
    tooling.stock_removed_mm3 = cylVol + std::max(0.0, conV - cylInChamfer);
    tooling.est_cycle_time_s  = std::max(1.0, throughLen / 50.0);
    tooling.extra = {
        { "iso_spec",         "ISO 273" },
        { "two_cut_sequence", {
            { { "tool_type", "drill" },        { "tool_dia_mm", clrDia } },
            { { "tool_type", "chamfer_mill" }, { "leg_mm",      leg    } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::bolt_hole_metric_spec applied: {} {} → clr {:.2f} mm",
                  in.thread_size, in.fit_class, clrDia);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy: look for through cylindrical faces whose diameter matches an
// ISO 273 entry (within 0.15 mm).  Metadata-replay fallback: scan features.

namespace {

struct CylInfo
{
    int    faceIdx = -1;
    double radius  = 0.0;
    gp_Ax1 axis;
    gp_Pnt entryCenter;
};

struct ConeInfo
{
    int    faceIdx = -1;
    gp_Ax1 axis;
    double rSmall = 0.0;
    double rLarge = 0.0;
    double semiAngleRad = 0.0;
    gp_Pnt cLarge;
};

bool sameAxisInfinite(const gp_Ax1& a, const gp_Ax1& b,
                      double angTolDeg = 0.5, double posTolMm = 0.05)
{
    const gp_Dir da = a.Direction(), db = b.Direction();
    const double dot = std::abs(da.X()*db.X() + da.Y()*db.Y() + da.Z()*db.Z());
    if (dot < std::cos(angTolDeg * M_PI / 180.0)) return false;
    gp_Vec v(a.Location(), b.Location());
    gp_Vec axV(da.X(), da.Y(), da.Z());
    gp_Vec perp = v - axV * v.Dot(axV);
    return perp.Magnitude() < posTolMm;
}

std::vector<CylInfo> collectCylinders(const Workpiece& wp)
{
    std::vector<CylInfo> out;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();

        CylInfo info;
        info.faceIdx = fIdx;
        info.radius  = cyl.Radius();
        info.axis    = cyl.Axis();

        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(c.Radius() - info.radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.empty()) continue;

        const gp_Dir adir = info.axis.Direction();
        auto proj = [&](const gp_Pnt& p) {
            return (p.X() - info.axis.Location().X()) * adir.X() +
                   (p.Y() - info.axis.Location().Y()) * adir.Y() +
                   (p.Z() - info.axis.Location().Z()) * adir.Z();
        };
        const auto minIt = std::min_element(centers.begin(), centers.end(),
            [&](const gp_Pnt& a, const gp_Pnt& b) { return proj(a) < proj(b); });
        info.entryCenter = *minIt;
        out.push_back(info);
    }
    return out;
}

std::vector<ConeInfo> collectCones(const Workpiece& wp)
{
    std::vector<ConeInfo> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface surf(wp.face(i));
        if (surf.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cn = surf.Cone();
        ConeInfo d;
        d.faceIdx = i;
        d.axis    = cn.Axis();
        d.semiAngleRad = std::abs(cn.SemiAngle());
        std::vector<std::pair<double, gp_Pnt>> circs;
        for (TopExp_Explorer exp(wp.face(i), TopAbs_EDGE); exp.More(); exp.Next()) {
            BRepAdaptor_Curve crv(TopoDS::Edge(exp.Current()));
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(d.axis.Direction())) - 1.0) > 1e-3)
                continue;
            circs.emplace_back(c.Radius(), c.Location());
        }
        if (circs.size() < 2) continue;
        auto mn = std::min_element(circs.begin(), circs.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });
        auto mx = std::max_element(circs.begin(), circs.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });
        if (std::abs(mx->first - mn->first) < 0.05) continue;
        d.rSmall = mn->first;
        d.rLarge = mx->first;
        d.cLarge = mx->second;
        out.push_back(d);
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Pass 1: metadata replay (fast and exact).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.99;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Pass 2: geometric fallback — match cylinder dia to ISO 273; if a
    // coaxial conical face is present at the cyl entry, recover the chamfer
    // (leg, angle) directly from the cone semi-angle and radius pair.
    const auto cyls  = collectCylinders(wp);
    const auto cones = collectCones(wp);
    for (const auto& c : cyls) {
        const double dia = 2.0 * c.radius;
        const ReverseMatch m = closestSpecMatch(dia, 0.15);
        if (m.size.empty()) continue;

        // Find a coaxial chamfer cone whose small radius matches the cyl.
        double chamfer_leg = 0.5;
        double chamfer_angle_deg = 30.0;
        int    cone_face_id = -1;
        bool   chamfer_found = false;
        for (const auto& cn : cones) {
            if (!sameAxisInfinite(c.axis, cn.axis)) continue;
            if (std::abs(cn.rSmall - c.radius) > 0.2) continue;
            chamfer_leg       = std::max(0.0, cn.rLarge - cn.rSmall);
            chamfer_angle_deg = 90.0 - cn.semiAngleRad * 180.0 / M_PI;
            if (chamfer_angle_deg < 15.0 || chamfer_angle_deg > 75.0) continue;
            cone_face_id      = cn.faceIdx;
            chamfer_found     = true;
            break;
        }

        const gp_Dir adir = c.axis.Direction();
        // Back-mapped spec-table key: "<size>-<fit>" e.g. "M6-medium".
        const std::string specKeyTable = m.size + std::string("-") + m.fit;
        json params = {
            { "position_x_mm",     c.entryCenter.X() },
            { "position_y_mm",     c.entryCenter.Y() },
            { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
            { "thread_size",       m.size },
            { "fit_class",         m.fit },
            { "spec_key_table",    specKeyTable },
            { "chamfer_size_mm",   chamfer_leg },
            { "chamfer_angle_deg", chamfer_angle_deg },
            { "clearance_dia_mm",  dia },
            { "fit_match_err_mm",  m.err },
        };
        // Confidence higher when chamfer cone was geometrically matched.
        const double base = chamfer_found ? 0.78 : 0.65;
        const double conf = base - std::min(0.2, m.err);
        json matched = {
            { "cyl_face_id",     c.faceIdx },
            { "iso",             "ISO 273" },
            { "spec_key_table",  specKeyTable },
            { "source",          chamfer_found ? "geometric_fallback" : "cyl_only" },
            { "chamfer_found",   chamfer_found },
            { "fit_err_mm",      m.err },
        };
        if (chamfer_found) matched["cone_face_id"] = cone_face_id;
        out.push_back(RecognizedFeature{ kSkillId, params, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::bolt_hole_metric_spec
