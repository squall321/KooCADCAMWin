// @lat: [[engine/skills#gmt_bezel_24hr]]

#include "gmt_bezel_24hr.hpp"

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

namespace koocadcam::skill::gmt_bezel_24hr {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.outer_dia_mm <= 0.0 || in.inner_dia_mm <= 0.0 ||
        in.height_mm <= 0.0 || in.marker_dia_mm <= 0.0 ||
        in.marker_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gmt_bezel_24hr: all dimensions must be > 0");
        return r;
    }
    if (in.inner_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-BEZEL-GEOM", "error",
              "gmt_bezel_24hr: outer_dia must exceed inner_dia");
    }
    if (in.hour_marker_count < 12 || in.hour_marker_count > 24) {
        r.add("DFM-MARKER-COUNT", "error",
              "gmt_bezel_24hr: hour_marker_count " +
              std::to_string(in.hour_marker_count) +
              " outside GMT/world-time range [12, 24]");
    }
    if (in.height_mm < 0.3) {
        r.add("DFM-BEZEL-HEIGHT", "error",
              "gmt_bezel_24hr: height < 0.3 mm minimum");
    }
    if (in.marker_depth_mm > in.height_mm * 0.95) {
        r.add("DFM-MARKER-DEPTH", "warning",
              "gmt_bezel_24hr: marker_depth may exceed bezel thickness");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain:
//   1. annular bezel ring cut
//   2..N+1. hour marker cuts every 360°/hour_marker_count around outer rim.
//
// All cutters FUSED then ONE single cut.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gmt_bezel_24hr DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double rOuter = in.outer_dia_mm / 2.0;
    const double rInner = in.inner_dia_mm / 2.0;
    const double rMarker = rOuter - in.marker_dia_mm * 0.5;

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

    // ── Sub-features 2..N+1: hour marker cuts ────────────────────────────
    const gp_Ax1 rotAxis(axisLoc, axisDir);
    const gp_Pnt seedStart(axisLoc.X() + rMarker, axisLoc.Y(), topZ + kOverhang);
    const gp_Ax2 seedAx(seedStart, cutDir);
    const TopoDS_Shape seedMarker = pr::cylinder(
        seedAx, in.marker_dia_mm / 2.0,
        in.marker_depth_mm + kOverhang);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(in.hour_marker_count) + 1);
    cutters.push_back(bezelRing);
    const double stepDeg = 360.0 / in.hour_marker_count;
    for (int i = 0; i < in.hour_marker_count; ++i) {
        const double aRad = i * stepDeg * M_PI / 180.0;
        gp_Trsf rot;
        rot.SetRotation(rotAxis, aRad);
        BRepBuilderAPI_Transform xform(seedMarker, rot, true);
        if (!xform.IsDone())
            throw SkillError("gmt_bezel_24hr: marker transform failed");
        cutters.push_back(xform.Shape());
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const double bezelVol = M_PI * (rOuter * rOuter - rInner * rInner) * in.height_mm;
    const double markerVol = M_PI * (in.marker_dia_mm / 2.0) * (in.marker_dia_mm / 2.0)
                           * in.marker_depth_mm * in.hour_marker_count;
    const double totalRemoved = bezelVol + markerVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",       { axisLoc.X(), axisLoc.Y(), axisLoc.Z() } },
        { "case_axis_dir",       { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "outer_dia_mm",        in.outer_dia_mm },
        { "inner_dia_mm",        in.inner_dia_mm },
        { "height_mm",           in.height_mm },
        { "hour_marker_count",   in.hour_marker_count },
        { "marker_dia_mm",       in.marker_dia_mm },
        { "marker_depth_mm",     in.marker_depth_mm },
        { "top_z_mm",            topZ },
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
            { { "op", "hour_marker_pattern" },
              { "instance_count", in.hour_marker_count },
              { "step_deg",       360.0 / in.hour_marker_count },
              { "marker_dia_mm",  in.marker_dia_mm },
              { "marker_depth_mm", in.marker_depth_mm } },
        }) },
        { "outer_dia_mm",        in.outer_dia_mm },
        { "inner_dia_mm",        in.inner_dia_mm },
        { "hour_marker_count",   in.hour_marker_count },
        { "marker_step_deg",     360.0 / in.hour_marker_count },
        { "scale_kind",          "gmt_24hr" },
        { "derived_volume_removed_mm3", totalRemoved },
        { "cylindrical_face_count", 2 + in.hour_marker_count },
        { "annular_face_count",     1 },
        { "axis_dir",            { axisDir.X(), axisDir.Y(), axisDir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "annular_groove;drill";
    tooling.tool_dia_mm       = in.outer_dia_mm;
    tooling.tool_length_mm    = in.height_mm + 3.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 230.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(4.0, totalRemoved / 80.0);
    tooling.extra = {
        { "feature_standard", "GMT 24-hour dual-time bezel (15°/hour)" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::gmt_bezel_24hr applied: outerD={} markers={}",
                  in.outer_dia_mm, in.hour_marker_count);

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
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "hour_marker_count", f.pattern.value("hour_marker_count", 24) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::gmt_bezel_24hr
