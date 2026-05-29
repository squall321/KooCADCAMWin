// @lat: [[engine/skills#crown_stem_cavity_compound]]

#include "crown_stem_cavity_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::crown_stem_cavity_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// AS568 small cross-section catalog (-001 family) entries we accept.
// CS = nominal cord cross-section (mm).
constexpr std::array<double, 4> kAS568SmallCS { 0.74, 1.0, 1.42, 1.78 };

bool isStandardAS568CS(double cs_mm, double tol_mm = 0.05)
{
    for (double v : kAS568SmallCS)
        if (std::abs(v - cs_mm) < tol_mm) return true;
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.case_outer_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "crown_stem_cavity_compound: case_outer_radius_mm must be > 0");
    }
    if (in.cavity_dia_mm <= 0.0 || in.stem_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "crown_stem_cavity_compound: cavity_dia/stem_dia must be > 0");
        return r;
    }

    if (in.stem_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "crown_stem_cavity_compound: stem_dia " +
              std::to_string(in.stem_dia_mm) + " mm < min 0.8 mm");
    }

    if (in.cavity_dia_mm < in.stem_dia_mm + 0.8) {
        r.add("DFM-CROWN-SHELF", "error",
              "crown_stem_cavity_compound: cavity_dia " +
              std::to_string(in.cavity_dia_mm) +
              " mm must be ≥ stem_dia + 0.8 mm (" +
              std::to_string(in.stem_dia_mm + 0.8) +
              ") — insufficient radial shelf for O-ring");
    }

    if (in.cavity_depth_mm < 1.5 || in.cavity_depth_mm > 5.0) {
        r.add("DFM-CROWN-DEPTH", "warning",
              "crown_stem_cavity_compound: cavity_depth " +
              std::to_string(in.cavity_depth_mm) +
              " mm outside typical [1.5, 5.0] mm waterproof crown range");
    }

    if (!isStandardAS568CS(in.o_ring_cs_mm)) {
        r.add("DFM-ORING-CS", "error",
              "crown_stem_cavity_compound: o_ring_cs " +
              std::to_string(in.o_ring_cs_mm) +
              " mm not in AS568 -001 family (0.74/1.0/1.42/1.78)");
    }

    // O-ring depth rule: AS568 design = 0.75 × CS ± tolerance.  We allow
    // 0.25 × CS to 0.85 × CS as practical range.
    if (in.o_ring_depth_mm < 0.25 * in.o_ring_cs_mm ||
        in.o_ring_depth_mm > 0.85 * in.o_ring_cs_mm) {
        r.add("DFM-ORING-DEPTH", "warning",
              "crown_stem_cavity_compound: o_ring_depth " +
              std::to_string(in.o_ring_depth_mm) +
              " mm outside AS568 design rule (0.25-0.85 × CS)");
    }

    // Case wall margin at crown port: case_outer_radius - cavity_depth must
    // leave enough wall thickness (≥ 1.0 mm).
    if (!wp.shape().IsNull()) {
        const double wall = in.case_outer_radius_mm - in.cavity_depth_mm;
        if (wall < 1.0) {
            r.add("DFM-CROWN-WALL", "error",
                  "crown_stem_cavity_compound: wall thickness " +
                  std::to_string(wall) + " mm < 1.0 mm — port cuts through case");
        }
    }

    if (in.stem_total_length_mm <= in.cavity_depth_mm) {
        r.add("DFM-INPUT", "error",
              "crown_stem_cavity_compound: stem_total_length must exceed cavity_depth");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "crown_stem_cavity_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Cavity axis: radial inward at angle_deg.
    const double a = in.angle_deg * M_PI / 180.0;
    const gp_Dir radialOut(std::cos(a), std::sin(a), 0.0);
    const gp_Dir radialIn (-radialOut.X(), -radialOut.Y(), 0.0);

    // Case surface entry point.
    const gp_Pnt surfacePt(
        in.case_axis.Location().X() + in.case_outer_radius_mm * radialOut.X(),
        in.case_axis.Location().Y() + in.case_outer_radius_mm * radialOut.Y(),
        in.port_center_z_mm);

    constexpr double kOverhang = 0.5;
    // ── Sub-feature 1: cavity cylinder ───────────────────────────────────
    // Start just outside the case (overhang) and go inward by cavity_depth.
    const gp_Pnt cavityStart(
        surfacePt.X() - radialIn.X() * kOverhang,
        surfacePt.Y() - radialIn.Y() * kOverhang,
        surfacePt.Z());
    const gp_Ax2 cavityAx(cavityStart, radialIn);
    const TopoDS_Shape cavityCyl = pr::cylinder(
        cavityAx, in.cavity_dia_mm / 2.0, in.cavity_depth_mm + kOverhang);

    // ── Sub-feature 2: stem through-hole (coaxial with cavity) ───────────
    const TopoDS_Shape stemCyl = pr::cylinder(
        cavityAx, in.stem_dia_mm / 2.0, in.stem_total_length_mm + kOverhang);

    // Cavity + stem are CONCENTRIC OVERLAPPING cylinders → fuse first then cut.
    const TopoDS_Shape stemFused = pr::fuse(cavityCyl, stemCyl);

    // ── Sub-feature 3: O-ring groove (annular ring inside cavity wall) ──
    // Groove is annular: outer radius = cavity_dia/2 + o_ring_depth, inner
    // radius = cavity_dia/2.  Centered axially at cavity_depth - o_ring_offset.
    const double rCavity      = in.cavity_dia_mm / 2.0;
    const double rGrooveOuter = rCavity + in.o_ring_depth_mm;
    const double rGrooveInner = rCavity - 0.02;   // bias to avoid coplanar
    // Place at o_ring_offset into the cavity.
    const gp_Pnt grooveStart(
        surfacePt.X() + radialIn.X() * (in.o_ring_offset_mm - in.o_ring_cs_mm / 2.0),
        surfacePt.Y() + radialIn.Y() * (in.o_ring_offset_mm - in.o_ring_cs_mm / 2.0),
        surfacePt.Z());
    const gp_Ax2 grooveAx(grooveStart, radialIn);
    const TopoDS_Shape groove =
        pr::annularRing(grooveAx, rGrooveOuter, rGrooveInner, in.o_ring_cs_mm);

    // Fuse groove with the cavity-stem cutter then single cut.
    const TopoDS_Shape totalCutter = pr::fuse(stemFused, groove);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), totalCutter);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",         { in.case_axis.Location().X(),
                                     in.case_axis.Location().Y(),
                                     in.case_axis.Location().Z() } },
        { "case_outer_radius_mm",  in.case_outer_radius_mm },
        { "angle_deg",             in.angle_deg },
        { "port_center_z_mm",      in.port_center_z_mm },
        { "cavity_dia_mm",         in.cavity_dia_mm },
        { "cavity_depth_mm",       in.cavity_depth_mm },
        { "stem_dia_mm",           in.stem_dia_mm },
        { "stem_total_length_mm",  in.stem_total_length_mm },
        { "o_ring_cs_mm",          in.o_ring_cs_mm },
        { "o_ring_depth_mm",       in.o_ring_depth_mm },
        { "o_ring_offset_mm",      in.o_ring_offset_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },
        { "subfeatures", json::array({
            { { "op", "cavity_cut" },     { "dia_mm", in.cavity_dia_mm }, { "depth_mm", in.cavity_depth_mm } },
            { { "op", "stem_through" },   { "dia_mm", in.stem_dia_mm },   { "depth_mm", in.stem_total_length_mm } },
            { { "op", "o_ring_groove" },  { "cs_mm", in.o_ring_cs_mm },    { "depth_mm", in.o_ring_depth_mm } },
        }) },
        { "cavity_dia_mm",       in.cavity_dia_mm },
        { "stem_dia_mm",         in.stem_dia_mm },
        { "radial_shelf_mm",     (in.cavity_dia_mm - in.stem_dia_mm) / 2.0 },
        { "axis_dir",            { radialIn.X(), radialIn.Y(), radialIn.Z() } },
        { "angle_deg",           in.angle_deg },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;bore;groove_tool";
    tooling.tool_dia_mm       = in.cavity_dia_mm;
    tooling.tool_length_mm    = in.stem_total_length_mm + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 230.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double cavityVol = M_PI * (in.cavity_dia_mm/2) * (in.cavity_dia_mm/2)
                             * in.cavity_depth_mm;
    const double stemVol   = M_PI * (in.stem_dia_mm/2)   * (in.stem_dia_mm/2)
                             * (in.stem_total_length_mm - in.cavity_depth_mm);
    const double grooveVol = M_PI *
        (rGrooveOuter * rGrooveOuter - rCavity * rCavity) * in.o_ring_cs_mm;
    tooling.stock_removed_mm3 = cavityVol + stemVol + grooveVol;
    tooling.est_cycle_time_s  = std::max(3.0, tooling.stock_removed_mm3 / 100.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },       { "tool_dia_mm", in.stem_dia_mm },
              { "depth_mm", in.stem_total_length_mm } },
            { { "tool_type", "bore" },        { "tool_dia_mm", in.cavity_dia_mm },
              { "depth_mm", in.cavity_depth_mm } },
            { { "tool_type", "groove_tool" }, { "width_mm", in.o_ring_cs_mm },
              { "depth_mm", in.o_ring_depth_mm } },
        } },
        { "feature_standard", "AS568 -001 family O-ring crown seal" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::crown_stem_cavity_compound applied: cav={} stem={} oring_cs={}",
                  in.cavity_dia_mm, in.stem_dia_mm, in.o_ring_cs_mm);

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
            { "source", "metadata_replay" },
            { "is_compound", true },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::crown_stem_cavity_compound
