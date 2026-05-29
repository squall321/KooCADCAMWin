// @lat: [[engine/skills#x_ring_groove]]

#include "x_ring_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::x_ring_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Parker Q-series X-ring CS table ──────────────────────────────────────
namespace {

struct QEntry { const char* code; double cs_mm; };

constexpr std::array<QEntry, 5> kQTable {{
    { "Q1", 1.78 },
    { "Q2", 2.62 },
    { "Q3", 3.53 },
    { "Q4", 5.33 },
    { "Q5", 6.99 },
}};

}  // namespace

double crossSectionForQ(const std::string& q_code)
{
    for (const auto& e : kQTable)
        if (q_code == e.code) return e.cs_mm;
    return 0.0;
}

double recommendedXDepthFor(double cs_mm) { return 0.78 * cs_mm; }
double recommendedXWidthFor(double cs_mm) { return 1.70 * cs_mm; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.x_ring_size.empty()) {
        r.add("DFM-XRING", "error",
              "x_ring_groove: x_ring_size is empty");
        return r;
    }
    const double cs = crossSectionForQ(in.x_ring_size);
    if (cs <= 0.0) {
        r.add("DFM-XRING", "error",
              "x_ring_groove: unknown Q-series code '" + in.x_ring_size +
              "' (supported: Q1/Q2/Q3/Q4/Q5)");
        return r;
    }

    if (in.mean_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "x_ring_groove: mean_dia_mm must be > 0");
    }
    if (in.bore_or_shaft_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "x_ring_groove: bore_or_shaft_dia_mm must be > 0");
    }

    const double depth = recommendedXDepthFor(cs);
    const double shaftR = in.bore_or_shaft_dia_mm / 2.0;
    if (shaftR - depth < 1.0) {
        r.add("DFM-XRING-WALL", "error",
              "x_ring_groove: groove would leave wall < 1.0 mm "
              "(shaftR=" + std::to_string(shaftR) +
              ", depth=" + std::to_string(depth) + ")");
    }

    // mean_dia must lie between the groove bottom and shaft surface.
    const double meanR = in.mean_dia_mm / 2.0;
    if (meanR > shaftR + 1e-3) {
        r.add("DFM-INPUT", "error",
              "x_ring_groove: mean_dia_mm > shaft_dia (invalid)");
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double w = recommendedXWidthFor(cs);
        const double zLow  = in.position_z_mm - w / 2.0;
        const double zHigh = in.position_z_mm + w / 2.0;
        if (zLow < zMin - 1e-3 || zHigh > zMax + 1e-3) {
            r.add("DFM-INPUT", "error",
                  "x_ring_groove: groove zone outside stock Z extent");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "x_ring_groove DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double cs       = crossSectionForQ(in.x_ring_size);
    const double depth_G  = recommendedXDepthFor(cs);
    const double width_W  = recommendedXWidthFor(cs);
    const double sideAng  = 10.0;                            // 10° sidewalls
    const double sideRad  = sideAng * M_PI / 180.0;
    const double chamferH = std::min(0.8, width_W * 0.20);
    const double chamferRise = chamferH * std::tan(sideRad);

    const double shaftR    = in.bore_or_shaft_dia_mm / 2.0;
    const double grooveR   = shaftR - depth_G;
    const double overhangR = 0.5;

    const double zLow      = in.position_z_mm - width_W / 2.0;

    // ── Sub-feature 1: CENTER (full-width) GROOVE ───────────────────────
    const gp_Ax2 axCenter(gp_Pnt(0.0, 0.0, zLow), gp::DZ());
    const TopoDS_Shape centerGroove = pr::annularRing(
        axCenter, shaftR + overhangR, grooveR, width_W);

    // ── Sub-feature 2: UPPER SIDEWALL CHAMFER (10° draft) ────────────────
    //
    // A small cone-frustum cut where outer R goes from shaftR+overhangR
    // (above groove) down to the same — modelled as annularConeRing where
    // the INNER bound is a cone widening as it approaches the entry plane.
    const double zUp = in.position_z_mm + width_W / 2.0 - chamferH;
    const gp_Ax2 axUp(gp_Pnt(0.0, 0.0, zUp), gp::DZ());
    const TopoDS_Shape upperChamfer = pr::annularConeRing(
        axUp,
        /*outerR1Bottom=*/ shaftR + overhangR,
        /*outerR2Top  =*/ shaftR + overhangR,
        /*innerR      =*/ grooveR + chamferRise,
        /*height      =*/ chamferH);

    // ── Sub-feature 3: LOWER SIDEWALL CHAMFER (10° draft) ────────────────
    const double zDn = in.position_z_mm - width_W / 2.0;
    const gp_Ax2 axDn(gp_Pnt(0.0, 0.0, zDn), gp::DZ());
    const TopoDS_Shape lowerChamfer = pr::annularConeRing(
        axDn,
        /*outerR1Bottom=*/ shaftR + overhangR,
        /*outerR2Top  =*/ shaftR + overhangR,
        /*innerR      =*/ grooveR + chamferRise,
        /*height      =*/ chamferH);

    // ── Fuse + single cut ────────────────────────────────────────────────
    const TopoDS_Shape fused1   = pr::fuse(centerGroove, upperChamfer);
    const TopoDS_Shape fusedAll = pr::fuse(fused1, lowerChamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedAll);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "bore_or_shaft_dia_mm", in.bore_or_shaft_dia_mm },
        { "mean_dia_mm",          in.mean_dia_mm },
        { "position_z_mm",        in.position_z_mm },
        { "x_ring_size",          in.x_ring_size },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     3 },
        { "bore_or_shaft_dia_mm", in.bore_or_shaft_dia_mm },
        { "mean_dia_mm",          in.mean_dia_mm },
        { "position_z_mm",        in.position_z_mm },
        { "x_ring_size",          in.x_ring_size },
        { "cs_mm",                cs },
        { "groove_depth_mm",      depth_G },
        { "groove_width_mm",      width_W },
        { "groove_radius_mm",     grooveR },
        { "sidewall_angle_deg",   sideAng },
        { "axis",                 { 0.0, 0.0, 1.0 } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "lathe_groove_insert;form_tool";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = depth_G + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.035;
    tooling.stock_removed_mm3 = M_PI * (shaftR * shaftR - grooveR * grooveR) * width_W;
    tooling.est_cycle_time_s  = std::max(2.5, depth_G / 5.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "kind", "center_groove" },
              { "depth_mm", depth_G }, { "width_mm", width_W } },
            { { "kind", "upper_sidewall_chamfer" },
              { "angle_deg", sideAng } },
            { { "kind", "lower_sidewall_chamfer" },
              { "angle_deg", sideAng } },
        } },
        { "standard", "Parker Quad Seal Handbook §5-1" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::x_ring_groove applied: {} cs={} W={} G={}",
                  in.x_ring_size, cs, width_W, depth_G);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = feat.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source", "metadata_replay" },
            { "pattern", feat.pattern },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::x_ring_groove
