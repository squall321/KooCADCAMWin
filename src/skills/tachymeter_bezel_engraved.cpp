// @lat: [[engine/skills#tachymeter_bezel_engraved]]

#include "tachymeter_bezel_engraved.hpp"

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
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::tachymeter_bezel_engraved {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr std::array<int, 3> kMajorCountChoices { 12, 24, 60 };

bool isStandardMajorCount(int n) {
    for (int v : kMajorCountChoices) if (v == n) return true;
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;

    if (in.bezel_outer_dia_mm <= 0.0 || in.bezel_inner_dia_mm <= 0.0 ||
        in.engrave_depth_mm <= 0.0 || in.major_tick_width_mm <= 0.0 ||
        in.minor_tick_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tachymeter_bezel_engraved: all dimensions must be > 0");
        return r;
    }
    if (in.bezel_inner_dia_mm >= in.bezel_outer_dia_mm) {
        r.add("DFM-BEZEL-GEOM", "error",
              "tachymeter_bezel_engraved: bezel_outer_dia must exceed inner_dia");
        return r;
    }
    if (in.engrave_depth_mm < 0.05 || in.engrave_depth_mm > 0.8) {
        r.add("DFM-ENGRAVE-DEPTH", "error",
              "tachymeter_bezel_engraved: engrave_depth " +
              std::to_string(in.engrave_depth_mm) +
              " mm outside legible range [0.05, 0.8] mm");
    }
    if (!isStandardMajorCount(in.major_count)) {
        r.add("DFM-MAJOR-COUNT", "error",
              "tachymeter_bezel_engraved: major_count " +
              std::to_string(in.major_count) +
              " not in standard set {12, 24, 60}");
    }
    if (in.minor_count <= 0 || in.minor_count > 240) {
        r.add("DFM-MINOR-COUNT", "error",
              "tachymeter_bezel_engraved: minor_count " +
              std::to_string(in.minor_count) + " outside (0, 240]");
    }
    if (in.major_count > 0 && (in.minor_count % in.major_count) != 0) {
        r.add("DFM-MINOR-COUNT", "error",
              "tachymeter_bezel_engraved: minor_count " +
              std::to_string(in.minor_count) +
              " not divisible by major_count " +
              std::to_string(in.major_count));
    }
    // Clearance between adjacent ticks at inner radius (worst case = minors).
    const int totalTicks = in.major_count + in.minor_count;
    if (totalTicks > 0) {
        const double stepRad = 2.0 * M_PI / totalTicks;
        const double innerR  = in.bezel_inner_dia_mm / 2.0;
        const double chord   = 2.0 * innerR * std::sin(stepRad / 2.0);
        const double widest  = std::max(in.major_tick_width_mm, in.minor_tick_width_mm);
        const double clear   = chord - widest;
        if (clear < 0.3) {
            r.add("DFM-TICK-CLEARANCE", "error",
                  "tachymeter_bezel_engraved: tick clearance at inner Ø " +
                  std::to_string(clear) +
                  " mm < 0.3 mm — adjacent ticks merge");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Each tick is a thin rectangular box cut, oriented radially:
//   - DX = tick_width  (tangential)
//   - DY = ring_radial_extent (radial — across the ring width)
//   - DZ = engrave_depth (downward into top face)
// Seeded at angle 0, then rotated about case axis N times via gp_Trsf.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tachymeter_bezel_engraved DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double rOuter = in.bezel_outer_dia_mm / 2.0;
    const double rInner = in.bezel_inner_dia_mm / 2.0;
    const double radialExtent = rOuter - rInner;
    const double majorLen = radialExtent - in.major_tick_inset_mm;
    const double minorLen = radialExtent * 0.55;   // shorter than major

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = (in.top_z_mm > 0.0) ? in.top_z_mm : zMax;
    constexpr double kOverhang = 0.02;

    const gp_Pnt axisLoc = in.case_axis.Location();
    const gp_Dir axisDir = in.case_axis.Direction();

    // Seed tick at angle 0 (sitting on +X radial line).  Local frame: box(ax, dx, dy, dz)
    // with gp_Ax2(P, V, Vx): V = local Z, Vx = local X.
    //
    // We want:
    //   tick footprint sits on top face, going DOWN by engrave_depth → V = -Z
    //   DX = tick_width (tangential, along +Y at angle 0) → Vx = +Y
    //   DY = tick length (radial) — derived as Vx × V (right-handed) = +Y × -Z = +X (radial outward ✓)
    auto buildTickSeed = [&](double tickWidth, double tickLen) {
        const double yShift = -tickWidth / 2.0;
        const gp_Pnt origin(
            axisLoc.X() + rInner,           // start at inner radius on +X
            axisLoc.Y() + yShift,           // center tangentially
            topZ + kOverhang);
        const gp_Ax2 ax(origin,
                        gp_Dir(-axisDir.X(), -axisDir.Y(), -axisDir.Z()),
                        gp_Dir(0.0, 1.0, 0.0));
        return pr::box(ax, tickWidth, tickLen, in.engrave_depth_mm + kOverhang);
    };

    const TopoDS_Shape majorSeed = buildTickSeed(in.major_tick_width_mm, majorLen);
    const TopoDS_Shape minorSeed = buildTickSeed(in.minor_tick_width_mm, minorLen);

    const gp_Ax1 rotAxis(axisLoc, axisDir);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(in.major_count + in.minor_count));

    // Majors at every (360 / major_count) degrees.
    const double majorStep = 360.0 / in.major_count;
    for (int i = 0; i < in.major_count; ++i) {
        gp_Trsf rot;
        rot.SetRotation(rotAxis, i * majorStep * M_PI / 180.0);
        BRepBuilderAPI_Transform xf(majorSeed, rot, true);
        if (!xf.IsDone())
            throw SkillError("tachymeter_bezel_engraved: major tick transform failed");
        cutters.push_back(xf.Shape());
    }
    // Minors at every (360 / minor_count) degrees, skipping positions that
    // coincide with majors so we don't double-stamp the same angle.
    const double minorStep = 360.0 / in.minor_count;
    const int minorsPerMajor = in.minor_count / in.major_count;
    for (int j = 0; j < in.minor_count; ++j) {
        if ((j % minorsPerMajor) == 0) continue;  // skip major positions
        gp_Trsf rot;
        rot.SetRotation(rotAxis, j * minorStep * M_PI / 180.0);
        BRepBuilderAPI_Transform xf(minorSeed, rot, true);
        if (!xf.IsDone())
            throw SkillError("tachymeter_bezel_engraved: minor tick transform failed");
        cutters.push_back(xf.Shape());
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const double majorVol = in.major_tick_width_mm * majorLen *
                            in.engrave_depth_mm * in.major_count;
    const double effMinorCount = in.minor_count -
                                 (in.minor_count / std::max(1, minorsPerMajor));
    const double minorVol = in.minor_tick_width_mm * minorLen *
                            in.engrave_depth_mm * effMinorCount;
    const double totalRemoved = majorVol + minorVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",       { axisLoc.X(), axisLoc.Y(), axisLoc.Z() } },
        { "case_axis_dir",       { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "bezel_outer_dia_mm",  in.bezel_outer_dia_mm },
        { "bezel_inner_dia_mm",  in.bezel_inner_dia_mm },
        { "engrave_depth_mm",    in.engrave_depth_mm },
        { "major_count",         in.major_count },
        { "minor_count",         in.minor_count },
        { "major_tick_width_mm", in.major_tick_width_mm },
        { "minor_tick_width_mm", in.minor_tick_width_mm },
        { "top_z_mm",            topZ },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_watch_feature",    true },
        { "watch_feature_type",  "tachymeter_scale" },
        { "subfeature_count",    2 },
        { "subfeatures", json::array({
            { { "op", "major_radial_marks" },
              { "instance_count", in.major_count },
              { "step_deg",       360.0 / in.major_count },
              { "width_mm",       in.major_tick_width_mm },
              { "length_mm",      majorLen } },
            { { "op", "minor_radial_marks" },
              { "instance_count", static_cast<int>(effMinorCount) },
              { "step_deg",       360.0 / in.minor_count },
              { "width_mm",       in.minor_tick_width_mm },
              { "length_mm",      minorLen } },
        }) },
        { "bezel_outer_dia_mm",  in.bezel_outer_dia_mm },
        { "bezel_inner_dia_mm",  in.bezel_inner_dia_mm },
        { "major_count",         in.major_count },
        { "minor_count",         in.minor_count },
        { "derived_volume_removed_mm3", totalRemoved },
        { "axis_dir",            { axisDir.X(), axisDir.Y(), axisDir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "engrave_mill;laser_mark";
    tooling.tool_dia_mm       = std::min(in.major_tick_width_mm,
                                         in.minor_tick_width_mm);
    tooling.tool_length_mm    = in.engrave_depth_mm + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.02;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(3.0, totalRemoved / 30.0);
    tooling.extra = {
        { "feature_standard",
          "chronograph tachymeter (Speedmaster/Daytona-style) engraved scale" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::tachymeter_bezel_engraved applied: outerD={} majors={} minors={}",
        in.bezel_outer_dia_mm, in.major_count, in.minor_count);

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
            { "major_count", f.pattern.value("major_count", 12) },
            { "minor_count", f.pattern.value("minor_count", 48) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::tachymeter_bezel_engraved
