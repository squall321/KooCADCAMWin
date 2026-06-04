// @lat: [[engine/skills#screw_down_caseback_6_notches]]

#include "screw_down_caseback_6_notches.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::screw_down_caseback_6_notches {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.caseback_dia_mm <= 0.0 || in.thread_pitch_mm <= 0.0 ||
        in.thread_depth_mm <= 0.0 || in.notch_width_mm <= 0.0 ||
        in.notch_depth_mm <= 0.0 || in.notch_radial_extent_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "screw_down_caseback_6_notches: all dimensions must be > 0");
        return r;
    }
    if (in.thread_pitch_mm < 0.5 || in.thread_pitch_mm > 2.0) {
        r.add("DFM-THREAD-PITCH", "error",
              "screw_down_caseback_6_notches: thread_pitch " +
              std::to_string(in.thread_pitch_mm) +
              " mm outside [0.5, 2.0] (M-thread fine/coarse range)");
    }
    if (in.notch_count < 4 || in.notch_count > 8) {
        r.add("DFM-NOTCH-COUNT", "error",
              "screw_down_caseback_6_notches: notch_count " +
              std::to_string(in.notch_count) +
              " outside watch-tool standard [4, 8]");
    }
    if (in.notch_depth_mm > in.caseback_dia_mm / 4.0) {
        r.add("DFM-NOTCH-DEPTH", "warning",
              "screw_down_caseback_6_notches: notch_depth > caseback_dia/4 — weakens caseback");
    }
    if (in.thread_depth_mm < in.thread_pitch_mm * 2.0) {
        r.add("DFM-THREAD-ENGAGE", "warning",
              "screw_down_caseback_6_notches: thread_depth < 2 × pitch — insufficient thread engagement");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain:
//   1. cylindrical thread relief (caseback_dia, depth = thread_depth)
//   2..N+1. rectangular spanner notches (box cuts) around the outer perimeter,
//           rotated about the case axis.  Each notch goes radially outward
//           from the caseback rim.
//
// All cutters FUSED into one compound then ONE cut against the workpiece.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "screw_down_caseback_6_notches DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bottomZ = (in.bottom_z_mm != 0.0) ? in.bottom_z_mm : zMin;
    constexpr double kOverhang = 0.05;

    const gp_Pnt axisLoc = in.case_axis.Location();
    const gp_Dir axisDir = in.case_axis.Direction();
    // Cut goes UPWARD from bottom into the case body.
    const gp_Pnt threadStart(axisLoc.X(), axisLoc.Y(), bottomZ - kOverhang);
    const gp_Ax2 threadAx(threadStart, axisDir);

    // ── Sub-feature 1: cylindrical thread relief ─────────────────────────
    const double rCaseback = in.caseback_dia_mm / 2.0;
    const TopoDS_Shape threadCyl = pr::cylinder(
        threadAx, rCaseback, in.thread_depth_mm + kOverhang);

    // First cut: remove the thread relief cylinder from the workpiece as its
    // OWN Boolean op (separate from the spanner notches).  Packing the large
    // thread cylinder and 6 small overlapping notch boxes into a single
    // compound caused OCCT's BRepAlgoAPI_Cut to silently produce a no-op
    // (the notches sit ON the cylinder's surface, triggering self-intersection
    // diagnostics in the compound's fusion preprocessor).  Splitting in two
    // is faster AND correct.
    const TopoDS_Shape afterThread = pr::cut(wp.shape(), threadCyl);

    // ── Sub-features 2..N+1: spanner notches around perimeter ────────────
    // Each notch = a small box that straddles the rim (rCaseback - 0.1 mm)
    // and extends radially OUTWARD into the rim material, giving a
    // case-opener tool radial leverage.
    //
    // gp_Ax2(P, V, Vx) — V is local Z, Vx is local X.  pr::box(ax, dx, dy, dz)
    // makes DX along Vx, DY along V×Vx, DZ along V.  With V = +Z, Vx = +X:
    //   DX → +X = radial,  DY → +Y = tangential,  DZ → +Z = axial.
    //
    // Seed at angle 0 (+X-axis from case centre):
    //   X: [rCaseback - 0.1, rCaseback - 0.1 + notch_radial_extent]
    //   Y: [-notch_width/2, +notch_width/2]
    //   Z: [bottomZ - overhang, bottomZ - overhang + notch_depth + overhang]
    const gp_Ax1 rotAxis(axisLoc, axisDir);
    const gp_Pnt notchOrigin(
        axisLoc.X() + rCaseback - 0.1,
        axisLoc.Y() - in.notch_width_mm / 2.0,
        bottomZ - kOverhang);
    const gp_Ax2 notchAx(notchOrigin, gp::DZ(), gp::DX());
    const TopoDS_Shape seedNotch = pr::box(
        notchAx,
        in.notch_radial_extent_mm,         // DX = radial outward
        in.notch_width_mm,                  // DY = tangential
        in.notch_depth_mm + kOverhang);     // DZ = axial into body

    std::vector<TopoDS_Shape> notchCutters;
    notchCutters.reserve(static_cast<size_t>(in.notch_count));
    const double stepDeg = 360.0 / in.notch_count;
    for (int i = 0; i < in.notch_count; ++i) {
        const double aRad = i * stepDeg * M_PI / 180.0;
        gp_Trsf rot;
        rot.SetRotation(rotAxis, aRad);
        BRepBuilderAPI_Transform xform(seedNotch, rot, true);
        if (!xform.IsDone())
            throw SkillError("screw_down_caseback_6_notches: notch transform failed");
        notchCutters.push_back(xform.Shape());
    }

    // Second cut: remove all 6 notches from the post-thread workpiece.
    // 60° apart at radius ≈15 mm → chord ≈15 mm between centres vs width
    // 1.5 mm — comfortably non-overlapping, safe for compound cut.
    const TopoDS_Shape newShape = pr::cutMany(afterThread, notchCutters);

    const double threadVol = M_PI * rCaseback * rCaseback * in.thread_depth_mm;
    const double notchVol  = in.notch_radial_extent_mm *
                             in.notch_width_mm *
                             in.notch_depth_mm * in.notch_count;
    const double totalRemoved = threadVol + notchVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",       { axisLoc.X(), axisLoc.Y(), axisLoc.Z() } },
        { "case_axis_dir",       { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "caseback_dia_mm",     in.caseback_dia_mm },
        { "thread_pitch_mm",     in.thread_pitch_mm },
        { "thread_depth_mm",     in.thread_depth_mm },
        { "notch_count",         in.notch_count },
        { "notch_width_mm",      in.notch_width_mm },
        { "notch_depth_mm",      in.notch_depth_mm },
        { "notch_radial_extent_mm", in.notch_radial_extent_mm },
        { "bottom_z_mm",         bottomZ },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_watch_feature",    true },
        { "subfeature_count",    2 },
        { "subfeatures", json::array({
            { { "op", "thread_relief_cyl" },
              { "dia_mm",   in.caseback_dia_mm },
              { "depth_mm", in.thread_depth_mm },
              { "pitch_mm", in.thread_pitch_mm } },
            { { "op", "spanner_notch_pattern" },
              { "instance_count", in.notch_count },
              { "step_deg",       360.0 / in.notch_count },
              { "notch_width_mm",         in.notch_width_mm },
              { "notch_depth_mm",         in.notch_depth_mm },
              { "notch_radial_extent_mm", in.notch_radial_extent_mm } },
        }) },
        { "caseback_dia_mm",     in.caseback_dia_mm },
        { "notch_count",         in.notch_count },
        { "notch_step_deg",      360.0 / in.notch_count },
        { "thread_pitch_mm",     in.thread_pitch_mm },
        { "derived_volume_removed_mm3", totalRemoved },
        { "cylindrical_face_count", 1 },                  // thread relief
        { "rect_pocket_count",      in.notch_count },     // notch boxes
        { "axis_dir",            { axisDir.X(), axisDir.Y(), axisDir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "thread_mill;end_mill";
    tooling.tool_dia_mm       = in.caseback_dia_mm;
    tooling.tool_length_mm    = in.thread_depth_mm + 3.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(6.0, totalRemoved / 80.0);
    tooling.extra = {
        { "feature_standard", "screw-down watch caseback, ISO 6425 water-resistant" },
        { "tool_sequence", {
            { { "tool_type", "thread_mill" }, { "dia_mm", in.caseback_dia_mm },
              { "pitch_mm", in.thread_pitch_mm } },
            { { "tool_type", "end_mill" },    { "dia_mm", in.notch_width_mm },
              { "notch_count", in.notch_count } },
        } },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::screw_down_caseback_6_notches applied: caseD={} notches={}",
                  in.caseback_dia_mm, in.notch_count);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",      "metadata_replay" },
            { "is_compound", true },
            { "notch_count", f.pattern.value("notch_count", 6) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::screw_down_caseback_6_notches
