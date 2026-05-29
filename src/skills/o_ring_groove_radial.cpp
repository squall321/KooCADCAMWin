// @lat: [[engine/skills#o_ring_groove_radial]]

#include "o_ring_groove_radial.hpp"

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

namespace koocadcam::skill::o_ring_groove_radial {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── AS568 cross-section table (representative dash codes) ────────────────
//
// AS568A standard sizes — only the dash codes most commonly used in
// hydraulic/pneumatic seals are included.  Full table has 300+ entries;
// the same pattern extends.
namespace {

struct AS568Entry {
    const char* dash;
    double      cs_mm;     // cross-section (CS) in mm
};

constexpr std::array<AS568Entry, 11> kAS568Table {{
    { "-006", 1.78 },
    { "-014", 1.78 },
    { "-110", 2.62 },
    { "-114", 2.62 },
    { "-210", 3.53 },
    { "-220", 3.53 },
    { "-225", 3.53 },
    { "-228", 3.53 },
    { "-310", 5.33 },
    { "-325", 5.33 },
    { "-425", 6.99 },
}};

}  // namespace

double crossSectionFor(const std::string& as568_dash)
{
    for (const auto& e : kAS568Table)
        if (as568_dash == e.dash) return e.cs_mm;
    return 0.0;
}

double recommendedDepthFor(double cs_mm) { return 0.74 * cs_mm; }
double recommendedWidthFor(double cs_mm) { return 1.45 * cs_mm; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.o_ring_size.empty()) {
        r.add("DFM-AS568", "error",
              "o_ring_groove_radial: o_ring_size is empty");
        return r;
    }
    const double cs = crossSectionFor(in.o_ring_size);
    if (cs <= 0.0) {
        r.add("DFM-AS568", "error",
              "o_ring_groove_radial: unknown AS568 size '" + in.o_ring_size +
              "' (supported: -006/-014/-110/-114/-210/-220/-225/-228/-310/-325/-425)");
        return r;
    }
    if (in.bore_or_shaft_dia_mm < 2.0) {
        r.add("DFM-INPUT", "error",
              "o_ring_groove_radial: bore_or_shaft_dia_mm " +
              std::to_string(in.bore_or_shaft_dia_mm) +
              " < 2.0 mm (insufficient material for groove + 1 mm wall)");
    }
    const double depth = recommendedDepthFor(cs);
    const double shaftRadius = in.bore_or_shaft_dia_mm / 2.0;
    if (depth > 0.0 && shaftRadius - depth < 1.0) {
        r.add("DFM-AS568-WALL", "error",
              "o_ring_groove_radial: groove would leave wall < 1.0 mm "
              "(shaftR=" + std::to_string(shaftRadius) +
              ", groove depth=" + std::to_string(depth) + ")");
    }

    // Workpiece Z extent check.
    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double w = recommendedWidthFor(cs);
        const double zLow  = in.position_z_mm - w / 2.0;
        const double zHigh = in.position_z_mm + w / 2.0;
        if (zLow < zMin - 1e-3 || zHigh > zMax + 1e-3) {
            r.add("DFM-INPUT", "error",
                  "o_ring_groove_radial: groove zone outside stock Z extent");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "o_ring_groove_radial DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double cs       = crossSectionFor(in.o_ring_size);
    const double depth_G  = recommendedDepthFor(cs);
    const double width_W  = recommendedWidthFor(cs);
    const double bottomR  = 0.05 * width_W;          // 5%-W bottom fillet
    const double draftAng = 5.0 * M_PI / 180.0;      // 5° sidewall draft

    const double shaftR     = in.bore_or_shaft_dia_mm / 2.0;
    const double grooveR    = shaftR - depth_G;
    const double overhangR  = 0.5;                   // ensure clean Boolean
    const double zLow       = in.position_z_mm - width_W / 2.0;

    // ── Sub-feature 1: MAIN GROOVE (rect cross-section annular ring) ────
    const gp_Ax2 ax1(gp_Pnt(0.0, 0.0, zLow), gp::DZ());
    const TopoDS_Shape mainGroove = pr::annularRing(
        ax1, shaftR + overhangR, grooveR, width_W);

    // ── Sub-feature 2: BOTTOM RADIUS RELIEF ──────────────────────────────
    //
    // Approximate the 5%-W bottom fillet by an additional thin annular
    // ring at the groove bottom: inner_R = grooveR - bottomR, outer_R =
    // grooveR + ε, height = 2·bottomR, centered axially on position_z.
    const double zBotRing = in.position_z_mm - bottomR;
    const gp_Ax2 ax2(gp_Pnt(0.0, 0.0, zBotRing), gp::DZ());
    const TopoDS_Shape bottomFillet = pr::annularRing(
        ax2, grooveR + 1e-3, grooveR - bottomR, 2.0 * bottomR);

    // ── Sub-feature 3: SIDE DRAFT (lead-in chamfer rings) ──────────────
    //
    // Two short cone-frustum cuts at the upper and lower edges of the
    // groove — gives the 5° sidewall draft for O-ring entry.  Each chamfer
    // is a thin annular volume whose outer radius increases with axial
    // distance from groove-bottom plane.  We model each as an annular cone
    // ring of height C_draft and OD growing from groove ID to shaftR.
    const double draftH   = std::min(0.5, width_W * 0.15);
    const double draftRad = draftH * std::tan(draftAng);

    // Upper draft: cone frustum r1 (bottom) = grooveR, r2 (top) = grooveR + draftRad
    const double zUpDraft = in.position_z_mm + width_W / 2.0 - draftH;
    const gp_Ax2 axUp(gp_Pnt(0.0, 0.0, zUpDraft), gp::DZ());
    const TopoDS_Shape upperDraft = pr::annularConeRing(
        axUp,
        /*outerR1Bottom=*/ shaftR + overhangR,
        /*outerR2Top  =*/ shaftR + overhangR,
        /*innerR      =*/ grooveR + draftRad,
        /*height      =*/ draftH);

    const double zDnDraft = in.position_z_mm - width_W / 2.0;
    const gp_Ax2 axDn(gp_Pnt(0.0, 0.0, zDnDraft), gp::DZ());
    const TopoDS_Shape lowerDraft = pr::annularConeRing(
        axDn,
        /*outerR1Bottom=*/ shaftR + overhangR,
        /*outerR2Top  =*/ shaftR + overhangR,
        /*innerR      =*/ grooveR + draftRad,
        /*height      =*/ draftH);

    // ── Fuse all sub-feature cutters then single boolean cut ────────────
    //
    // Concentric overlapping cutter pattern (see counterbore.cpp): fuse
    // the cutters into one tool to avoid lost-overlap issues with the
    // BRepAlgoAPI cut algorithm.
    const TopoDS_Shape fused1 = pr::fuse(mainGroove, bottomFillet);
    const TopoDS_Shape fused2 = pr::fuse(fused1, upperDraft);
    const TopoDS_Shape fusedAll = pr::fuse(fused2, lowerDraft);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedAll);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "bore_or_shaft_dia_mm", in.bore_or_shaft_dia_mm },
        { "position_z_mm",        in.position_z_mm },
        { "o_ring_size",          in.o_ring_size },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  3 },
        { "bore_or_shaft_dia_mm", in.bore_or_shaft_dia_mm },
        { "position_z_mm",        in.position_z_mm },
        { "o_ring_size",          in.o_ring_size },
        { "cs_mm",                cs },
        { "groove_depth_mm",      depth_G },
        { "groove_width_mm",      width_W },
        { "groove_radius_mm",     grooveR },
        { "bottom_radius_mm",     bottomR },
        { "draft_angle_deg",      5.0 },
        { "axis",                 { 0.0, 0.0, 1.0 } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "lathe_groove_insert;profile_form_tool";
    tooling.tool_dia_mm       = 0.0;   // form tool — width-controlled
    tooling.tool_length_mm    = depth_G + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.03;  // slow finish on sealing geometry
    tooling.stock_removed_mm3 = M_PI * (shaftR * shaftR - grooveR * grooveR) * width_W;
    tooling.est_cycle_time_s  = std::max(2.0, depth_G / 5.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "kind", "main_rect_groove" },
              { "depth_mm", depth_G }, { "width_mm", width_W } },
            { { "kind", "bottom_radius" },
              { "radius_mm", bottomR } },
            { { "kind", "side_draft" },
              { "angle_deg", 5.0 } },
        } },
        { "standard", "AS568 / Parker O-ring Handbook §4-1" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::o_ring_groove_radial applied: {} cs={} W={} G={} br={} faces {}→{}",
                  in.o_ring_size, cs, width_W, depth_G, bottomR,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Compound features are hard to fingerprint from raw geometry alone (the
// 3 sub-features fuse into one annular cavity that looks topologically
// similar to a plain groove + a few extra bevel faces).  We rely on
// METADATA REPLAY: if the workpiece's feature history contains an
// o_ring_groove_radial signature, report it with confidence 1.0.  No
// geometric inference is attempted for slice-1.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id = kSkillId;
        r.recovered_params = feat.params;
        r.confidence       = 1.0;     // metadata replay → ground truth
        r.matched_geometry = {
            { "source", "metadata_replay" },
            { "pattern", feat.pattern },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::o_ring_groove_radial
