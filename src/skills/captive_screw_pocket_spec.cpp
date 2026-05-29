// @lat: [[engine/skills#captive_screw_pocket_spec]]

#include "captive_screw_pocket_spec.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::captive_screw_pocket_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── DIN 6799 retaining-ring groove geometry table ───────────────────────
//
// Each row: thread_size, ISO-273-medium clearance dia, DIN 6799 groove dia
// (on the shaft side, but reused here for cavity-side groove), ring width.
// Approximation values for the consumer-electronics + structural set.

namespace {

struct CaptiveSpec
{
    const char* size;
    double      clearance_dia_mm;   // ISO 273 medium
    double      groove_dia_mm;      // DIN 6799 retaining-ring groove
    double      groove_width_mm;    // DIN 6799 ring thickness
};

constexpr std::array<CaptiveSpec, 11> kDin6799Table {{
    { "M3",   3.4,  4.0,  0.40 },
    { "M4",   4.5,  5.0,  0.60 },
    { "M5",   5.5,  6.2,  0.70 },
    { "M6",   6.6,  7.4,  0.80 },
    { "M8",   9.0, 10.0,  1.00 },
    { "M10", 11.0, 12.0,  1.10 },
    { "M12", 14.0, 15.0,  1.30 },
    { "M16", 18.0, 19.0,  1.60 },
    { "M20", 22.0, 23.0,  2.00 },
    { "M24", 26.0, 27.5,  2.30 },
    { "M30", 33.0, 34.5,  2.60 },
}};

const CaptiveSpec* findSpec(const std::string& size)
{
    for (const auto& e : kDin6799Table) {
        if (size == e.size) return &e;
    }
    return nullptr;
}

}  // namespace

double clearanceDiameterFor(const std::string& thread_size)
{
    if (const CaptiveSpec* s = findSpec(thread_size)) return s->clearance_dia_mm;
    return 0.0;
}
double grooveDiameterFor(const std::string& thread_size)
{
    if (const CaptiveSpec* s = findSpec(thread_size)) return s->groove_dia_mm;
    return 0.0;
}
double grooveWidthFor(const std::string& thread_size)
{
    if (const CaptiveSpec* s = findSpec(thread_size)) return s->groove_width_mm;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const CaptiveSpec* s = findSpec(in.thread_size);
    if (!s) {
        r.add("DFM-METRIC-UNKNOWN", "error",
              "captive_screw_pocket_spec: unknown thread_size '" + in.thread_size +
              "' (DIN 6799 table supports M3..M30)");
        return r;
    }
    if (s->groove_width_mm < 0.2) {
        r.add("DFM-CAPTIVE-WIDTH", "error",
              "captive_screw_pocket_spec: groove width " +
              std::to_string(s->groove_width_mm) +
              " < 0.2 mm — below DIN 6799 minimum ring thickness");
    }
    if (in.groove_offset_mm < 1.0) {
        r.add("DFM-CAPTIVE-OFFSET", "warning",
              "captive_screw_pocket_spec: groove_offset " +
              std::to_string(in.groove_offset_mm) +
              " < 1.0 mm — wall between face and groove may chip during assembly");
    }
    if (in.chamfer_size_mm < 0.0) {
        r.add("DFM-INPUT", "error", "chamfer_size_mm must be >= 0");
    }

    // Stock thickness must contain groove_offset + groove_width + bit of meat.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness =
        std::abs(in.axis_dir.X()) * (xMax - xMin) +
        std::abs(in.axis_dir.Y()) * (yMax - yMin) +
        std::abs(in.axis_dir.Z()) * (zMax - zMin);
    const double needed = in.groove_offset_mm + s->groove_width_mm + 1.0;
    if (thickness > 0.0 && thickness < needed) {
        r.add("DFM-CAPTIVE-DEPTH", "error",
              "captive_screw_pocket_spec: stock thickness " +
              std::to_string(thickness) + " < required " + std::to_string(needed));
    }
    return r;
}

// ── Synthesis: 3-cut compound ────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "captive_screw_pocket_spec DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const CaptiveSpec* s = findSpec(in.thread_size);
    const double clrDia    = s->clearance_dia_mm;
    const double clrR      = clrDia / 2.0;
    const double grooveDia = s->groove_dia_mm;
    const double grooveR   = grooveDia / 2.0;
    const double grooveW   = s->groove_width_mm;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const gp_Dir adir = in.axis_dir;
    const double kOverhang = 0.05;

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
    const double throughLen = bboxDiag + 2.0 * kOverhang;

    // CUT 1: clearance through-cylinder.
    const TopoDS_Shape clearance = pr::cylinder(toolAx, clrR, throughLen);

    // CUT 2: entry chamfer (45° standard).
    const double leg = std::max(in.chamfer_size_mm, 0.0);
    const double chamferDepth = (leg > 0.0) ? leg : 0.0;
    const double chamferBigR  = clrR + leg;
    TopoDS_Shape chamferTool;
    if (leg > 1e-6 && chamferDepth > 1e-6) {
        chamferTool = pr::coneFrustum(toolAx, chamferBigR, clrR, chamferDepth);
    }

    // CUT 3: DIN 6799 retaining-ring groove — annular ring at offset.
    gp_Pnt grooveStart(
        toolStart.X() + adir.X() * (kOverhang + in.groove_offset_mm),
        toolStart.Y() + adir.Y() * (kOverhang + in.groove_offset_mm),
        toolStart.Z() + adir.Z() * (kOverhang + in.groove_offset_mm));
    const gp_Ax2 grooveAx(grooveStart, adir);
    const TopoDS_Shape grooveTool = pr::cylinder(grooveAx, grooveR, grooveW);

    // Fuse and single-cut.
    TopoDS_Shape fused = clearance;
    if (!chamferTool.IsNull()) fused = pr::fuse(fused, chamferTool);
    fused = pr::fuse(fused, grooveTool);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size",      in.thread_size },
        { "chamfer_size_mm",  in.chamfer_size_mm },
        { "groove_offset_mm", in.groove_offset_mm },
        { "clearance_dia_mm", clrDia },
        { "groove_dia_mm",    grooveDia },
        { "groove_width_mm",  grooveW },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           3 },
        { "thread_size",                in.thread_size },
        { "clearance_dia_mm",           clrDia },
        { "groove_dia_mm",              grooveDia },
        { "groove_width_mm",            grooveW },
        { "groove_offset_mm",           in.groove_offset_mm },
        { "cylindrical_face_count",     2 },
        { "conical_face_count",         (leg > 0.0) ? 1 : 0 },
        { "annular_step_face_count",    2 },  // two annular steps at groove
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;chamfer_mill;groove_tool";
    tooling.tool_dia_mm      = clrDia;
    tooling.tool_length_mm   = throughLen + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double cylVol    = M_PI * clrR * clrR * throughLen;
    const double grooveVol = M_PI * (grooveR * grooveR - clrR * clrR) * grooveW;
    tooling.stock_removed_mm3 = cylVol + grooveVol;
    tooling.est_cycle_time_s  = std::max(2.0, throughLen / 40.0 + 1.0);
    tooling.extra = {
        { "spec",  "DIN 6799 + ISO 273 medium" },
        { "three_cut_sequence", {
            { { "tool_type", "drill" },        { "tool_dia_mm", clrDia    } },
            { { "tool_type", "chamfer_mill" }, { "leg_mm",      leg       } },
            { { "tool_type", "groove_tool" },  { "groove_dia_mm", grooveDia },
                                                { "width_mm",      grooveW   },
                                                { "offset_mm",     in.groove_offset_mm } },
        } },
        { "captive_screw_function", "retaining-ring (E-clip) traps screw head" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::captive_screw_pocket_spec applied: {} clr {:.2f} "
                  "groove {:.2f}×{:.2f}",
                  in.thread_size, clrDia, grooveDia, grooveW);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: pair of coaxial cylinders where the larger one is
// SHORTER (the groove) at an offset from the entry face.  This matches the
// counterbore.cpp pair-detection pattern.

namespace {

struct CylInfo
{
    int    faceIdx = -1;
    double radius  = 0.0;
    gp_Ax1 axis;
    gp_Pnt topCenter;
    gp_Pnt botCenter;
    double length = 0.0;
};

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
        if (centers.size() < 2) continue;

        const gp_Dir adir = info.axis.Direction();
        auto proj = [&](const gp_Pnt& p) {
            return (p.X() - info.axis.Location().X()) * adir.X() +
                   (p.Y() - info.axis.Location().Y()) * adir.Y() +
                   (p.Z() - info.axis.Location().Z()) * adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) {
            return proj(a) < proj(b);
        };
        info.topCenter = *std::min_element(centers.begin(), centers.end(), cmp);
        info.botCenter = *std::max_element(centers.begin(), centers.end(), cmp);
        info.length    = info.topCenter.Distance(info.botCenter);
        out.push_back(info);
    }
    return out;
}

bool sameAxis(const CylInfo& a, const CylInfo& b)
{
    const gp_Dir da = a.axis.Direction();
    const gp_Dir db = b.axis.Direction();
    const double dot = std::abs(da.X()*db.X() + da.Y()*db.Y() + da.Z()*db.Z());
    if (dot < std::cos(0.5 * M_PI / 180.0)) return false;
    const gp_Pnt& pa = a.axis.Location();
    const gp_Pnt& pb = b.axis.Location();
    gp_Vec v(pa, pb);
    gp_Vec axV(da.X(), da.Y(), da.Z());
    gp_Vec perp = v - axV * v.Dot(axV);
    return perp.Magnitude() < 1e-3;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

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

    // Geometric heuristic: find a coaxial pair where larger cyl is short
    // (the groove) and small cyl extends most of the way through.
    const auto cyls = collectCylinders(wp);
    std::vector<bool> used(cyls.size(), false);

    for (size_t i = 0; i < cyls.size(); ++i) {
        if (used[i]) continue;
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (used[j] || !sameAxis(cyls[i], cyls[j])) continue;

            const CylInfo& small = (cyls[i].radius < cyls[j].radius) ? cyls[i] : cyls[j];
            const CylInfo& large = (cyls[i].radius < cyls[j].radius) ? cyls[j] : cyls[i];
            if (large.radius <= small.radius) continue;

            // For a captive pocket the LARGE cylinder is the GROOVE → short.
            if (large.length >= small.length * 0.5) continue;

            const double clrDia    = 2.0 * small.radius;
            const double grooveDia = 2.0 * large.radius;

            // Find matching DIN 6799 row.
            const CaptiveSpec* match = nullptr;
            double bestErr = 0.20;
            for (const auto& e : kDin6799Table) {
                const double err = std::abs(e.clearance_dia_mm - clrDia) +
                                   std::abs(e.groove_dia_mm    - grooveDia);
                if (err < bestErr) { bestErr = err; match = &e; }
            }
            if (!match) continue;

            const gp_Dir adir = small.axis.Direction();
            json params = {
                { "position_x_mm",   small.topCenter.X() },
                { "position_y_mm",   small.topCenter.Y() },
                { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
                { "thread_size",     match->size },
                { "clearance_dia_mm", clrDia },
                { "groove_dia_mm",    grooveDia },
                { "groove_width_mm",  large.length },
            };
            out.push_back(RecognizedFeature{
                kSkillId, params, 0.85,
                { { "clearance_face_id", small.faceIdx },
                  { "groove_face_id",    large.faceIdx },
                  { "spec",              "DIN 6799" } }
            });
            used[i] = used[j] = true;
            break;
        }
    }
    return out;
}

}  // namespace koocadcam::skill::captive_screw_pocket_spec
