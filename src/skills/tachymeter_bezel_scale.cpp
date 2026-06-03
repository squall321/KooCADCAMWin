// @lat: [[engine/skills#tachymeter_bezel_scale]]

#include "tachymeter_bezel_scale.hpp"

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

namespace koocadcam::skill::tachymeter_bezel_scale {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Tachymeter scale: theta_deg = 21600 / speed_kmh, modulo 360.
// We sample marking_count points logarithmically from 60 to 400 km/h.
double tachymeterAngleDeg(double speed_kmh)
{
    double a = 21600.0 / speed_kmh;
    while (a >= 360.0) a -= 360.0;
    while (a < 0.0)    a += 360.0;
    return a;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.outer_dia_mm <= 0.0 || in.inner_dia_mm <= 0.0 ||
        in.height_mm <= 0.0 || in.marking_depth_mm <= 0.0 ||
        in.marking_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tachymeter_bezel_scale: all dimensions must be > 0");
        return r;
    }
    if (in.inner_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-BEZEL-GEOM", "error",
              "tachymeter_bezel_scale: outer_dia must exceed inner_dia");
    }
    if (in.marking_count < 10 || in.marking_count > 240) {
        r.add("DFM-MARKING-COUNT", "error",
              "tachymeter_bezel_scale: marking_count " +
              std::to_string(in.marking_count) + " outside [10, 240]");
    }
    if (in.marking_depth_mm > in.height_mm * 0.8) {
        r.add("DFM-MARKING-DEPTH", "warning",
              "tachymeter_bezel_scale: marking_depth > 0.8 × bezel height — may breakthrough");
    }
    if (in.height_mm < 0.3) {
        r.add("DFM-BEZEL-HEIGHT", "error",
              "tachymeter_bezel_scale: height < 0.3 mm minimum");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain:
//   1. annular bezel ring cut (outer − inner cylinder, depth = height_mm)
//   2..N+1. shallow radial line marks at logarithmic tachymeter angles
//           (60 km/h .. 400 km/h, marking_count samples).  Each mark is a
//           small radial cylinder cut whose axis is +Z, placed at the mid-
//           radius of the annular ring.
//
// All cutters FUSED then ONE single cut against workpiece (matches
// sapphire_glass_seat pattern).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tachymeter_bezel_scale DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double rOuter = in.outer_dia_mm / 2.0;
    const double rInner = in.inner_dia_mm / 2.0;
    const double rMid   = (rOuter + rInner) / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = (in.top_z_mm > 0.0) ? in.top_z_mm : zMax;
    constexpr double kOverhang = 0.05;

    const gp_Pnt axisLoc = in.case_axis.Location();
    const gp_Dir axisDir = in.case_axis.Direction();
    const gp_Dir cutDir(-axisDir.X(), -axisDir.Y(), -axisDir.Z());
    const gp_Pnt bezelStart(axisLoc.X(), axisLoc.Y(), topZ + kOverhang);
    const gp_Ax2 bezelAx(bezelStart, cutDir);

    // ── Sub-feature 1: annular bezel ring ────────────────────────────────
    const TopoDS_Shape bezelRing = pr::annularRing(
        bezelAx, rOuter, rInner, in.height_mm + kOverhang);

    // ── Sub-features 2..N+1: tachymeter marks ────────────────────────────
    // Each mark = thin radial cut.  We approximate it as a small cylinder
    // whose axis is the case axis (vertical), located at rMid along the
    // bezel radius and offset to its angular position.  Marks are sampled
    // logarithmically in speed (60 km/h .. 400 km/h).
    const double speedLo = 60.0;
    const double speedHi = 400.0;
    const double logLo   = std::log(speedLo);
    const double logHi   = std::log(speedHi);

    const gp_Ax1 rotAxis(axisLoc, axisDir);
    // Seed mark at angle 0, at rMid radius.
    const gp_Pnt seedStart(axisLoc.X() + rMid, axisLoc.Y(), topZ + kOverhang);
    const gp_Ax2 seedAx(seedStart, cutDir);
    const TopoDS_Shape seedMark = pr::cylinder(
        seedAx, in.marking_width_mm / 2.0,
        in.marking_depth_mm + kOverhang);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(in.marking_count) + 1);
    cutters.push_back(bezelRing);
    for (int i = 0; i < in.marking_count; ++i) {
        // Log spacing.
        const double t = (in.marking_count <= 1) ? 0.0
                       : static_cast<double>(i) / (in.marking_count - 1);
        const double speed = std::exp(logLo + t * (logHi - logLo));
        const double aDeg  = tachymeterAngleDeg(speed);
        const double aRad  = aDeg * M_PI / 180.0;
        gp_Trsf rot;
        rot.SetRotation(rotAxis, aRad);
        BRepBuilderAPI_Transform xform(seedMark, rot, true);
        if (!xform.IsDone())
            throw SkillError("tachymeter_bezel_scale: mark transform failed");
        cutters.push_back(xform.Shape());
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    // ── Volumes ──────────────────────────────────────────────────────────
    const double bezelVol = M_PI * (rOuter * rOuter - rInner * rInner) * in.height_mm;
    const double markVol  = M_PI * (in.marking_width_mm / 2.0) *
                            (in.marking_width_mm / 2.0) *
                            in.marking_depth_mm * in.marking_count;
    const double totalRemoved = bezelVol + markVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",     { axisLoc.X(), axisLoc.Y(), axisLoc.Z() } },
        { "case_axis_dir",     { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "outer_dia_mm",      in.outer_dia_mm },
        { "inner_dia_mm",      in.inner_dia_mm },
        { "height_mm",         in.height_mm },
        { "marking_depth_mm",  in.marking_depth_mm },
        { "marking_count",     in.marking_count },
        { "marking_width_mm",  in.marking_width_mm },
        { "top_z_mm",          topZ },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_watch_feature",    true },
        { "subfeature_count",    2 },
        { "subfeatures", json::array({
            { { "op", "annular_bezel_ring" },
              { "outer_dia_mm", in.outer_dia_mm },
              { "inner_dia_mm", in.inner_dia_mm },
              { "depth_mm",     in.height_mm } },
            { { "op", "tachymeter_radial_marks" },
              { "instance_count", in.marking_count },
              { "scale_range_kmh", { speedLo, speedHi } },
              { "marking_depth_mm", in.marking_depth_mm },
              { "marking_width_mm", in.marking_width_mm } },
        }) },
        { "outer_dia_mm",        in.outer_dia_mm },
        { "inner_dia_mm",        in.inner_dia_mm },
        { "marking_count",       in.marking_count },
        { "scale_kind",          "tachymeter" },
        { "scale_range_kmh",     { speedLo, speedHi } },
        { "derived_volume_removed_mm3", totalRemoved },
        { "cylindrical_face_count", 2 + in.marking_count },
        { "annular_face_count",     1 },
        { "axis_dir",            { axisDir.X(), axisDir.Y(), axisDir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "annular_groove;engrave";
    tooling.tool_dia_mm       = in.outer_dia_mm;
    tooling.tool_length_mm    = in.height_mm + 3.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(4.0, totalRemoved / 60.0);
    tooling.extra = {
        { "feature_standard", "tachymeter scale: theta_deg = 21600 / speed_kmh" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tachymeter_bezel_scale applied: outerD={} marks={}",
                  in.outer_dia_mm, in.marking_count);

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
            { "source",        "metadata_replay" },
            { "is_compound",   true },
            { "marking_count", f.pattern.value("marking_count", 60) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::tachymeter_bezel_scale
