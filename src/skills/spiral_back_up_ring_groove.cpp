// @lat: [[engine/skills#spiral_back_up_ring_groove]]

#include "spiral_back_up_ring_groove.hpp"

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

namespace koocadcam::skill::spiral_back_up_ring_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── AS568 table (shared with o_ring_groove_radial) ───────────────────────
namespace {

struct AS568Entry { const char* dash; double cs_mm; };

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

constexpr double kBackupWidthFactor = 0.5;  // Wb ≈ 0.5·Wo (spiral, single-side)

}  // namespace

double crossSectionFor(const std::string& as568_dash)
{
    for (const auto& e : kAS568Table)
        if (as568_dash == e.dash) return e.cs_mm;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.ring_size.empty()) {
        r.add("DFM-AS568", "error",
              "spiral_back_up_ring_groove: ring_size is empty");
        return r;
    }
    const double cs = crossSectionFor(in.ring_size);
    if (cs <= 0.0) {
        r.add("DFM-AS568", "error",
              "spiral_back_up_ring_groove: unknown AS568 size '" +
              in.ring_size + "'");
        return r;
    }
    if (in.position != "downstream" && in.position != "upstream") {
        r.add("DFM-INPUT", "error",
              "spiral_back_up_ring_groove: position must be 'downstream' or "
              "'upstream' (got '" + in.position + "')");
    }
    if (in.bore_or_shaft_dia_mm < 4.0) {
        r.add("DFM-INPUT", "error",
              "spiral_back_up_ring_groove: bore_or_shaft_dia_mm < 4.0 mm");
    }

    const double depthO  = 0.74 * cs;
    const double widthO  = 1.45 * cs;
    const double widthBu = kBackupWidthFactor * widthO;
    const double shaftR  = in.bore_or_shaft_dia_mm / 2.0;
    if (shaftR - depthO < 1.0) {
        r.add("DFM-AS568-WALL", "error",
              "spiral_back_up_ring_groove: O-ring groove would leave wall < 1 mm");
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        // Combined extent: O-ring width + interseptum 0.5 mm + back-up width.
        const double combined = widthO + 0.5 + widthBu;
        const double zLow  = in.position_z_mm - combined / 2.0;
        const double zHigh = in.position_z_mm + combined / 2.0;
        if (zLow < zMin - 1e-3 || zHigh > zMax + 1e-3) {
            r.add("DFM-INPUT", "error",
                  "spiral_back_up_ring_groove: combined groove zone outside stock Z");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spiral_back_up_ring_groove DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double cs      = crossSectionFor(in.ring_size);
    const double depthO  = 0.74 * cs;     // O-ring groove depth
    const double widthO  = 1.45 * cs;     // O-ring groove width
    const double depthBu = depthO;        // back-up slot same depth
    const double widthBu = kBackupWidthFactor * widthO;
    constexpr double kInterseptum = 0.5;  // 0.5 mm wall between rings

    const double shaftR    = in.bore_or_shaft_dia_mm / 2.0;
    const double grooveR   = shaftR - depthO;
    const double overhangR = 0.5;

    // ── Sub-feature 1: PRIMARY O-RING GROOVE ────────────────────────────
    const double zLowO = in.position_z_mm - widthO / 2.0;
    const gp_Ax2 axO(gp_Pnt(0.0, 0.0, zLowO), gp::DZ());
    const TopoDS_Shape oRingCut = pr::annularRing(
        axO, shaftR + overhangR, grooveR, widthO);

    // ── Sub-feature 2: BACK-UP RING SLOT (downstream or upstream) ────────
    //
    // Place adjacent to O-ring with `kInterseptum` mm wall.
    //   downstream → back-up sits at HIGHER z (positive offset).
    //   upstream   → back-up sits at LOWER z (negative offset).
    const double sign = (in.position == "downstream") ? +1.0 : -1.0;
    const double bzCenter = in.position_z_mm + sign * (widthO / 2.0 + kInterseptum + widthBu / 2.0);
    const double zLowBu = bzCenter - widthBu / 2.0;
    const gp_Ax2 axBu(gp_Pnt(0.0, 0.0, zLowBu), gp::DZ());
    const TopoDS_Shape backupCut = pr::annularRing(
        axBu, shaftR + overhangR, grooveR, widthBu);

    // ── Fuse + single cut ────────────────────────────────────────────────
    const TopoDS_Shape fused    = pr::fuse(oRingCut, backupCut);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "bore_or_shaft_dia_mm", in.bore_or_shaft_dia_mm },
        { "position_z_mm",        in.position_z_mm },
        { "ring_size",            in.ring_size },
        { "position",             in.position },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      2 },
        { "bore_or_shaft_dia_mm",  in.bore_or_shaft_dia_mm },
        { "position_z_mm",         in.position_z_mm },
        { "ring_size",             in.ring_size },
        { "position",              in.position },
        { "cs_mm",                 cs },
        { "oring_depth_mm",        depthO },
        { "oring_width_mm",        widthO },
        { "backup_depth_mm",       depthBu },
        { "backup_width_mm",       widthBu },
        { "groove_radius_mm",      grooveR },
        { "interseptum_mm",        kInterseptum },
        { "backup_center_z_mm",    bzCenter },
        { "axis",                  { 0.0, 0.0, 1.0 } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "lathe_groove_insert;form_tool";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = depthO + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 =
        M_PI * (shaftR * shaftR - grooveR * grooveR) * (widthO + widthBu);
    tooling.est_cycle_time_s  = std::max(3.0, depthO / 4.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "kind", "primary_oring_groove" },
              { "depth_mm", depthO }, { "width_mm", widthO } },
            { { "kind", "backup_ring_slot" },
              { "depth_mm", depthBu }, { "width_mm", widthBu },
              { "side", in.position } },
        } },
        { "standard", "Parker O-ring Handbook §5-1 (spiral back-up rings)" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::spiral_back_up_ring_groove applied: {} {} cs={} W_o={} W_b={}",
                  in.ring_size, in.position, cs, widthO, widthBu);

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

}  // namespace koocadcam::skill::spiral_back_up_ring_groove
