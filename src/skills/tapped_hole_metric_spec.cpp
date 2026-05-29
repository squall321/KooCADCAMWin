// @lat: [[engine/skills#tapped_hole_metric_spec]]

#include "tapped_hole_metric_spec.hpp"

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

namespace koocadcam::skill::tapped_hole_metric_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── ISO 965-1 tap drill table (1st-choice coarse pitch) ─────────────────

namespace {

struct ThreadPilot
{
    const char* size;
    double      pilot_dia_mm;
    double      pitch_mm;
};

constexpr std::array<ThreadPilot, 11> kIso965Table {{
    { "M3",   2.50, 0.50 },
    { "M4",   3.30, 0.70 },
    { "M5",   4.20, 0.80 },
    { "M6",   5.00, 1.00 },
    { "M8",   6.80, 1.25 },
    { "M10",  8.50, 1.50 },
    { "M12", 10.20, 1.75 },
    { "M16", 14.00, 2.00 },
    { "M20", 17.50, 2.50 },
    { "M24", 21.00, 3.00 },
    { "M30", 26.50, 3.50 },
}};

}  // namespace

double pilotDiameterFor(const std::string& thread_size)
{
    for (const auto& e : kIso965Table) {
        if (thread_size == e.size) return e.pilot_dia_mm;
    }
    return 0.0;
}

namespace {

double pitchFor(const std::string& thread_size)
{
    for (const auto& e : kIso965Table) {
        if (thread_size == e.size) return e.pitch_mm;
    }
    return 0.0;
}

std::string sizeForDiameter(double dia_mm, double tol_mm)
{
    const ThreadPilot* best = nullptr;
    double bestErr = tol_mm;
    for (const auto& e : kIso965Table) {
        const double err = std::abs(e.pilot_dia_mm - dia_mm);
        if (err < bestErr) { bestErr = err; best = &e; }
    }
    return best ? std::string(best->size) : std::string();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thread_size.empty()) {
        r.add("DFM-METRIC-UNKNOWN", "error",
              "tapped_hole_metric_spec: thread_size is empty");
        return r;
    }
    const double pilotDia = pilotDiameterFor(in.thread_size);
    if (pilotDia <= 0.0) {
        r.add("DFM-METRIC-UNKNOWN", "error",
              "tapped_hole_metric_spec: unknown thread_size '" + in.thread_size +
              "' (ISO 965 supports M3..M30, 11 sizes)");
        return r;
    }
    if (pilotDia < 0.8) {
        r.add("DFM-MARGIN", "error",
              "tapped_hole_metric_spec: pilot " + std::to_string(pilotDia) +
              " mm < min tap-drill 0.8 mm");
    }
    if (!in.through && in.tap_depth_mm <= 0.0) {
        r.add("DFM-TAP-DEPTH", "error",
              "tapped_hole_metric_spec: tap_depth_mm must be > 0 for blind tap");
    }
    if (in.pilot_extra_depth_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "tapped_hole_metric_spec: pilot_extra_depth_mm must be >= 0");
    }
    if (!in.through && in.tap_depth_mm > 0.0 && pilotDia > 0.0) {
        const double ratio = in.tap_depth_mm / pilotDia;
        if (ratio > 3.0) {
            r.add("DFM-TAP-PECK", "warning",
                  "tapped_hole_metric_spec: tap_depth/pilot " +
                  std::to_string(ratio) +
                  " > 3 — chip evacuation problematic, consider peck-tapping");
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
        std::string msg = "tapped_hole_metric_spec DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double pilotDia = pilotDiameterFor(in.thread_size);
    const double pilotR   = pilotDia / 2.0;
    const double pitch    = pitchFor(in.thread_size);

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

    // CUT 1: pilot drill (through or blind+extra).
    const double pilotLen = in.through
        ? bboxDiag + 2.0 * kOverhang
        : in.tap_depth_mm + in.pilot_extra_depth_mm + kOverhang;
    const TopoDS_Shape pilotTool = pr::cylinder(toolAx, pilotR, pilotLen);

    // CUT 2: entry chamfer at 60° included angle (30° from axis) per
    // standard tap-lead practice.
    const double leg = std::max(in.chamfer_size_mm, 0.0);
    const double chamferDepth = (leg > 0.0) ? leg / std::tan(30.0 * M_PI / 180.0) : 0.0;
    const double chamferBigR  = pilotR + leg;
    TopoDS_Shape chamferTool;
    if (leg > 1e-6 && chamferDepth > 1e-6) {
        chamferTool = pr::coneFrustum(toolAx, chamferBigR, pilotR, chamferDepth);
    }

    TopoDS_Shape fusedCutter = pilotTool;
    if (!chamferTool.IsNull()) {
        fusedCutter = pr::fuse(pilotTool, chamferTool);
    }
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size",          in.thread_size },
        { "tap_depth_mm",         in.tap_depth_mm },
        { "through",              in.through },
        { "chamfer_size_mm",      in.chamfer_size_mm },
        { "pilot_extra_depth_mm", in.pilot_extra_depth_mm },
        { "pilot_dia_mm",         pilotDia },
        { "pitch_mm",             pitch },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           2 },
        { "thread_size",                in.thread_size },
        { "pilot_dia_mm",               pilotDia },
        { "pitch_mm",                   pitch },
        { "tap_depth_mm",               in.tap_depth_mm },
        { "through",                    in.through },
        { "cylindrical_face_count",     1 },
        { "conical_face_count",         (leg > 0.0) ? 1 : 0 },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;tap";
    tooling.tool_dia_mm      = pilotDia;
    tooling.tool_length_mm   = pilotLen + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = M_PI * pilotR * pilotR * pilotLen;
    tooling.est_cycle_time_s  = std::max(1.0, pilotLen / 30.0);
    tooling.extra = {
        { "iso_spec", "ISO 965-1" },
        { "tap_thread", in.thread_size },
        { "two_cut_sequence", {
            { { "tool_type", "drill" }, { "tool_dia_mm", pilotDia } },
            { { "tool_type", "tap"   }, { "thread_size", in.thread_size },
                                        { "pitch_mm",    pitch },
                                        { "tap_depth_mm", in.tap_depth_mm } },
        } },
        { "note", "Tap thread modelled as metadata only (matches drill_and_tap convention)" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tapped_hole_metric_spec applied: {} pilot {:.2f} × {:.2f}",
                  in.thread_size, pilotDia, pilotLen);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Pass 1: metadata replay.
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

    // Pass 2: geometric — scan cylindrical faces, match dia to ISO 965.
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double dia = 2.0 * cyl.Radius();

        const std::string size = sizeForDiameter(dia, 0.10);
        if (size.empty()) continue;

        // Recover position via circular edges.
        gp_Pnt entryCenter = cyl.Axis().Location();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            entryCenter = crv.Circle().Location();
            break;
        }

        const gp_Dir adir = cyl.Axis().Direction();
        json params = {
            { "position_x_mm",     entryCenter.X() },
            { "position_y_mm",     entryCenter.Y() },
            { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
            { "thread_size",       size },
            { "tap_depth_mm",      0.0 },
            { "through",           true },
            { "pilot_dia_mm",      dia },
            { "pitch_mm",          pitchFor(size) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, params, 0.55,   // lower than drill_hole — overlaps
            { { "cyl_face_id", fIdx }, { "iso", "ISO 965-1" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::tapped_hole_metric_spec
