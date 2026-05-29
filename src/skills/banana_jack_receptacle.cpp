// @lat: [[engine/skills#banana_jack_receptacle]]

#include "banana_jack_receptacle.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::banana_jack_receptacle {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Geometric constants (IEC 61010 4 mm safety banana) ──────────────────
namespace {
constexpr double kThroughDia_mm     = 4.10;   // 4 mm plug + 0.10 clearance
constexpr double kCounterboreDia_mm = 9.00;   // front bezel recess Ø
constexpr double kCounterboreDepth_mm = 1.5;
constexpr double kRingGrooveDia_mm  = 7.00;   // DIN 471 internal Ø7 ring
constexpr double kRingGrooveDepth_mm = 0.90;  // DIN 471 spec
constexpr double kRingFromBack_mm   = 6.0;    // distance from back face
constexpr double kMinPanelThk_mm    = 8.0;    // counterbore + clearance + ring slot
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.count < 1 || in.count > 16) {
        r.add("DFM-INPUT", "error",
              "banana_jack_receptacle: count " + std::to_string(in.count) +
              " outside [1, 16]");
    }
    if (in.count > 1 && in.spacing_mm < 12.0) {
        r.add("DFM-INPUT", "error",
              "banana_jack_receptacle: spacing_mm " +
              std::to_string(in.spacing_mm) +
              " < 12 mm (Ø9 counterbores would overlap)");
    }

    // Through pilot must clear the DFM minimum drill rule.
    if constexpr (kThroughDia_mm < 0.8) {
        r.add("DFM-002", "error",
              "banana_jack_receptacle: through_dia < 0.8 mm");
    }

    // Panel thickness check — must be ≥ 8 mm to fit counterbore + ring slot.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness < kMinPanelThk_mm) {
        r.add("DFM-BANANA-PANEL", "error",
              "banana_jack_receptacle: panel thickness " +
              std::to_string(thickness) + " mm < " +
              std::to_string(kMinPanelThk_mm) +
              " mm (counterbore + ring slot won't fit)");
    }

    // DIN 471 verification: ring groove diameter and depth from the standard
    // 7 mm internal-ring row.
    if (std::abs(kRingGrooveDia_mm  - 7.00) > 0.05 ||
        std::abs(kRingGrooveDepth_mm - 0.90) > 0.05) {
        r.add("DFM-DIN471", "error",
              "banana_jack_receptacle: snap-ring groove not in DIN 471 catalog");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "banana_jack_receptacle DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.face);
    if (!entryId) throw SkillError("banana_jack_receptacle: face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;

    constexpr double kOverhang = 0.05;
    const double topZ = zMax;
    const double botZ = zMin;
    const gp_Dir downZ(0.0, 0.0, -1.0);

    // Row layout
    const double rad = in.direction_deg * M_PI / 180.0;
    const gp_Dir rowDir(std::cos(rad), std::sin(rad), 0.0);

    // Build cutters for each jack: through + counterbore + ring slot.
    std::vector<TopoDS_Shape> allCutters;
    allCutters.reserve(in.count * 3);

    for (int i = 0; i < in.count; ++i) {
        const double cx = in.position_x_mm + i * in.spacing_mm * rowDir.X();
        const double cy = in.position_y_mm + i * in.spacing_mm * rowDir.Y();

        // (1) Through pilot — cylinder from top down through full thickness.
        const gp_Pnt thruStart(cx, cy, topZ + kOverhang);
        const gp_Ax2 thruAx(thruStart, downZ);
        const TopoDS_Shape thruTool = pr::cylinder(thruAx, kThroughDia_mm / 2.0,
                                                   thickness + 2 * kOverhang);

        // (2) Counterbore — wider cylinder, only kCounterboreDepth_mm deep.
        const gp_Pnt cbStart(cx, cy, topZ + kOverhang);
        const gp_Ax2 cbAx(cbStart, downZ);
        const TopoDS_Shape cbTool = pr::cylinder(cbAx, kCounterboreDia_mm / 2.0,
                                                 kCounterboreDepth_mm + kOverhang);

        // (3) Rear ring-groove — annular ring centred at z = botZ + kRingFromBack
        //                        with outer = Ø7/2, inner = Ø4.1/2, height = 0.9
        const double ringZ = botZ + kRingFromBack_mm;
        const gp_Pnt ringOrigin(cx, cy, ringZ);
        const gp_Ax2 ringAx(ringOrigin, gp_Dir(0.0, 0.0, 1.0));
        // annularRing throws if inner ≥ outer; we have 4.1 < 7.0 OK.
        const TopoDS_Shape ringTool = pr::annularRing(
            ringAx,
            kRingGrooveDia_mm / 2.0,
            kThroughDia_mm    / 2.0,
            kRingGrooveDepth_mm);

        // Counterbore overlaps the through hole — fuse them concentrically
        // first (this mirrors the counterbore.cpp pattern), then combine
        // with the ring slot (which is annular and DOES intersect the
        // through pilot at its inner radius).  We fuse all three so we
        // perform one cut per jack.
        const TopoDS_Shape jackFused = pr::fuseMany(thruTool, { cbTool, ringTool });
        allCutters.push_back(jackFused);
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), allCutters);

    // ── Signature ───────────────────────────────────────────────────────
    json positions = json::array();
    for (int i = 0; i < in.count; ++i) {
        positions.push_back({
            { "x", in.position_x_mm + i * in.spacing_mm * rowDir.X() },
            { "y", in.position_y_mm + i * in.spacing_mm * rowDir.Y() },
        });
    }

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "count",          in.count },
        { "spacing_mm",     in.spacing_mm },
        { "direction_deg",  in.direction_deg },
    };

    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      3 },   // per-jack subfeatures
        { "jack_count",            in.count },
        { "spacing_mm",            in.spacing_mm },
        { "through_dia_mm",        kThroughDia_mm },
        { "counterbore_dia_mm",    kCounterboreDia_mm },
        { "counterbore_depth_mm",  kCounterboreDepth_mm },
        { "ring_groove_dia_mm",    kRingGrooveDia_mm },
        { "ring_groove_depth_mm",  kRingGrooveDepth_mm },
        { "ring_from_back_mm",     kRingFromBack_mm },
        { "positions",             positions },
        { "axis_dir",              { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "subfeatures",           {
            { { "kind", "through_hole" },     { "dia_mm", kThroughDia_mm } },
            { { "kind", "front_counterbore" }, { "dia_mm", kCounterboreDia_mm },
              { "depth_mm", kCounterboreDepth_mm } },
            { { "kind", "din471_snap_ring_groove" }, { "dia_mm", kRingGrooveDia_mm },
              { "depth_mm", kRingGrooveDepth_mm } },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;counterbore;internal_grooving_tool";
    tooling.tool_dia_mm      = kCounterboreDia_mm;
    tooling.tool_length_mm   = thickness * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;

    const double pilotR = kThroughDia_mm / 2.0;
    const double cbR    = kCounterboreDia_mm / 2.0;
    const double ringOuterR = kRingGrooveDia_mm / 2.0;
    const double perJackVol =
        M_PI * pilotR * pilotR * thickness +
        M_PI * (cbR * cbR - pilotR * pilotR) * kCounterboreDepth_mm +
        M_PI * (ringOuterR * ringOuterR - pilotR * pilotR) * kRingGrooveDepth_mm;
    tooling.stock_removed_mm3 = perJackVol * in.count;
    tooling.est_cycle_time_s  = std::max(2.0, in.count * 3.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" }, { "tool_dia_mm", kThroughDia_mm } },
            { { "tool_type", "counterbore" }, { "tool_dia_mm", kCounterboreDia_mm },
              { "depth_mm", kCounterboreDepth_mm } },
            { { "tool_type", "internal_grooving_tool" },
              { "groove_dia_mm", kRingGrooveDia_mm },
              { "depth_mm", kRingGrooveDepth_mm },
              { "note", "DIN 471 internal snap-ring Ø7" } },
        } },
        { "standard", "IEC 61010 4mm banana / DIN 471 internal Ø7" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::banana_jack_receptacle applied: count={} pitch={} faces {}→{}",
                  in.count, in.spacing_mm, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        const auto& p = f.pattern;
        json recovered = {
            { "position_x_mm", f.params.value("position_x_mm", 0.0) },
            { "position_y_mm", f.params.value("position_y_mm", 0.0) },
            { "count",         f.params.value("count",         1) },
            { "spacing_mm",    f.params.value("spacing_mm",   19.05) },
            { "direction_deg", f.params.value("direction_deg", 0.0) },
            { "axis_dir",      f.params.value("axis_dir",     json::array({0.0,0.0,-1.0})) },
        };
        json matched = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", p.value("subfeature_count", 0) },
            { "jack_count",       p.value("jack_count",       0) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 1.0, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::banana_jack_receptacle
