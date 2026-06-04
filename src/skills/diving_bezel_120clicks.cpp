// @lat: [[engine/skills#diving_bezel_120clicks]]

#include "diving_bezel_120clicks.hpp"

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

namespace koocadcam::skill::diving_bezel_120clicks {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;

    if (in.outer_dia_mm <= 0.0 || in.inner_dia_mm <= 0.0 ||
        in.height_mm <= 0.0 || in.notch_dia_mm <= 0.0 ||
        in.notch_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "diving_bezel_120clicks: all dimensions must be > 0");
        return r;
    }
    if (in.inner_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-BEZEL-GEOM", "error",
              "diving_bezel_120clicks: outer_dia (" +
              std::to_string(in.outer_dia_mm) + ") must exceed inner_dia (" +
              std::to_string(in.inner_dia_mm) + ")");
    }
    if (in.click_count < 60 || in.click_count > 240) {
        r.add("DFM-CLICK-COUNT", "error",
              "diving_bezel_120clicks: click_count " +
              std::to_string(in.click_count) +
              " outside ISO 6425 range [60, 240]");
    }
    if (in.height_mm < 0.3) {
        r.add("DFM-BEZEL-HEIGHT", "error",
              "diving_bezel_120clicks: height " +
              std::to_string(in.height_mm) + " mm < 0.3 mm minimum");
    }
    // Annular width / notch overlap rule.
    const double annularWidth = (in.outer_dia_mm - in.inner_dia_mm) / 2.0;
    if (annularWidth < in.notch_dia_mm * 0.5) {
        r.add("DFM-BEZEL-WIDTH", "warning",
              "diving_bezel_120clicks: annular width " +
              std::to_string(annularWidth) +
              " mm < notch_dia/2 — notches may overhang inner edge");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain:
//   1.  annular bezel ring (outer cyl + inner cyl return = annularRing prim)
//   2.. N+1. small notch cylinder cuts replicated around the outer rim every
//            360°/click_count degrees.  Each notch is a small cylinder placed
//            at the outer edge (radial distance = outer_radius - notch_dia/2)
//            and rotated about the case axis.
//
// All cutter shapes are FUSED into a single compound, then ONE cut against
// the workpiece (matches sapphire_glass_seat pattern).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "diving_bezel_120clicks DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double rOuter = in.outer_dia_mm / 2.0;
    const double rInner = in.inner_dia_mm / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = (in.top_z_mm > 0.0) ? in.top_z_mm : zMax;
    constexpr double kOverhang = 0.05;

    const gp_Pnt axisLoc = in.case_axis.Location();
    const gp_Dir axisDir = in.case_axis.Direction();
    // Bezel cut extends DOWN from topZ through height_mm of material.  We
    // anchor each cutter at its BOTTOM Z and build along +axisDir so that
    // BRepPrimAPI_MakeCylinder builds upward through the top.  Anchoring at
    // the bottom (rather than at topZ + overhang with a flipped axis)
    // avoids subtle gp_Ax2 inversion artifacts that, combined with
    // BRepAlgoAPI_Cut against a packed compound of 121 shapes, were
    // producing degenerate results.
    const double bezelBottomZ = topZ - in.height_mm;
    const gp_Pnt bezelStart(axisLoc.X(), axisLoc.Y(), bezelBottomZ);
    const gp_Ax2 bezelAx(bezelStart, axisDir);

    // ── Sub-feature 1: annular bezel ring ────────────────────────────────
    const TopoDS_Shape bezelRing = pr::annularRing(
        bezelAx, rOuter, rInner, in.height_mm + kOverhang);

    // First cut: remove the annular bezel ring from the workpiece.  Done as
    // its OWN Boolean op (separate from the click notches) — this is more
    // robust than packing 121 cutters into a single compound, which OCCT
    // sometimes degenerates on with overlapping coaxial shapes.
    const TopoDS_Shape afterBezel = pr::cut(wp.shape(), bezelRing);

    // ── Sub-features 2..N+1: click notches around outer rim ──────────────
    // Seed notch at angle = 0 (positive X-axis from case centre), placed at
    // the outer edge of the bezel rim, going DOWN through the TOP of the
    // case by notch_depth_mm.  Like the bezel, we anchor at the cutter
    // bottom (topZ − notch_depth) and build upward.
    const double rNotchCentre = rOuter;   // notch tangent to outer rim
    const double notchBottomZ = topZ - in.notch_depth_mm;
    const gp_Pnt notchSeedStart(
        axisLoc.X() + rNotchCentre,
        axisLoc.Y(),
        notchBottomZ);
    const gp_Ax2 notchSeedAx(notchSeedStart, axisDir);
    const TopoDS_Shape notchSeed = pr::cylinder(
        notchSeedAx, in.notch_dia_mm / 2.0,
        in.notch_depth_mm + kOverhang);

    const gp_Ax1 rotAxis(axisLoc, axisDir);

    std::vector<TopoDS_Shape> notchCutters;
    notchCutters.reserve(static_cast<size_t>(in.click_count));
    const double stepDeg = 360.0 / in.click_count;
    for (int i = 0; i < in.click_count; ++i) {
        const double aRad = i * stepDeg * M_PI / 180.0;
        gp_Trsf rot;
        rot.SetRotation(rotAxis, aRad);
        BRepBuilderAPI_Transform xform(notchSeed, rot, true);
        if (!xform.IsDone())
            throw SkillError("diving_bezel_120clicks: notch transform failed");
        notchCutters.push_back(xform.Shape());
    }

    // Second cut: remove all 120 click notches at once.  These are small,
    // non-overlapping (3° apart at radius 19 mm — chord ≈ 1.0 mm vs notch
    // Ø 0.6 mm), so a single compound cut is safe and fast.
    const TopoDS_Shape newShape = pr::cutMany(afterBezel, notchCutters);

    // ── Volumes for signature / tooling ──────────────────────────────────
    const double bezelVol = M_PI * (rOuter * rOuter - rInner * rInner) * in.height_mm;
    const double notchVol = M_PI * (in.notch_dia_mm / 2.0) * (in.notch_dia_mm / 2.0)
                          * in.notch_depth_mm * in.click_count;
    const double totalRemoved = bezelVol + notchVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",  { axisLoc.X(), axisLoc.Y(), axisLoc.Z() } },
        { "case_axis_dir",  { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "outer_dia_mm",   in.outer_dia_mm },
        { "inner_dia_mm",   in.inner_dia_mm },
        { "height_mm",      in.height_mm },
        { "click_count",    in.click_count },
        { "notch_dia_mm",   in.notch_dia_mm },
        { "notch_depth_mm", in.notch_depth_mm },
        { "top_z_mm",       topZ },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_watch_feature",    true },
        { "subfeature_count",    2 },          // logical: ring + clicks
        { "subfeatures", json::array({
            { { "op", "annular_bezel_seat" },
              { "outer_dia_mm", in.outer_dia_mm },
              { "inner_dia_mm", in.inner_dia_mm },
              { "depth_mm",     in.height_mm } },
            { { "op", "click_notch_pattern" },
              { "instance_count", in.click_count },
              { "step_deg",       360.0 / in.click_count },
              { "notch_dia_mm",   in.notch_dia_mm },
              { "notch_depth_mm", in.notch_depth_mm } },
        }) },
        { "outer_dia_mm",        in.outer_dia_mm },
        { "inner_dia_mm",        in.inner_dia_mm },
        { "annular_width_mm",    (in.outer_dia_mm - in.inner_dia_mm) / 2.0 },
        { "click_count",         in.click_count },
        { "click_step_deg",      360.0 / in.click_count },
        { "derived_volume_removed_mm3", totalRemoved },
        { "cylindrical_face_count", 2 + in.click_count }, // outer+inner+notches
        { "annular_face_count",     1 },                  // bezel bottom
        { "axis_dir",            { axisDir.X(), axisDir.Y(), axisDir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "annular_groove;drill";
    tooling.tool_dia_mm       = in.outer_dia_mm;
    tooling.tool_length_mm    = in.height_mm + 3.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 240.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(5.0, totalRemoved / 80.0);
    tooling.extra = {
        { "feature_standard", "ISO 6425 dive watch unidirectional bezel" },
        { "click_count_iso",  "120 standard (3° per click)" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::diving_bezel_120clicks applied: outerD={} innerD={} clicks={}",
                  in.outer_dia_mm, in.inner_dia_mm, in.click_count);

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
            { "source",       "metadata_replay" },
            { "is_compound",  true },
            { "click_count",  f.pattern.value("click_count", 120) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::diving_bezel_120clicks
