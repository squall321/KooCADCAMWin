// @lat: [[engine/skills#power_reserve_indicator_slot]]

#include "power_reserve_indicator_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::power_reserve_indicator_slot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.arc_radius_mm <= 0.0 || in.slot_width_mm <= 0.0 ||
        in.slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "power_reserve_indicator_slot: arc_radius/slot dims must be > 0");
        return r;
    }

    if (in.arc_end_deg <= in.arc_start_deg) {
        r.add("DFM-ARC", "error",
              "power_reserve_indicator_slot: arc_end_deg " +
              std::to_string(in.arc_end_deg) + " must exceed arc_start_deg " +
              std::to_string(in.arc_start_deg));
    }

    if (in.arc_segment_count < 4 || in.arc_segment_count > 48) {
        r.add("DFM-SEGMENTS", "error",
              "power_reserve_indicator_slot: arc_segment_count " +
              std::to_string(in.arc_segment_count) + " outside [4, 48]");
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double thickness = zMax - zMin;
        if (in.slot_depth_mm >= thickness) {
            r.add("DFM-DEPTH", "error",
                  "power_reserve_indicator_slot: slot_depth " +
                  std::to_string(in.slot_depth_mm) +
                  " mm >= stock thickness " + std::to_string(thickness) + " mm");
        }
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "power_reserve_indicator_slot DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    const gp_Ax1 spinAxis(gp_Pnt(cx, cy, 0.0), gp::DZ());
    const double startRad = in.arc_start_deg * M_PI / 180.0;

    // Seed segment box: radial element placed at the START angle, centred on
    // the arc radius.  Its footprint is (slot_width along tangent) ×
    // (radial chord length per segment) and depth = slot_depth.  We then
    // ROTATE copies about the dial centre to sweep the arc.
    const double sweepRad = (in.arc_end_deg - in.arc_start_deg) * M_PI / 180.0;
    const int N = in.arc_segment_count;
    const double stepRad = sweepRad / static_cast<double>(N);

    // Chord length of one segment (slightly overlapped so the union of cuts
    // is gap-free): arc length per segment plus a small overlap.
    const double segChord = in.arc_radius_mm * stepRad * 1.4 + in.slot_width_mm;

    // Seed at start angle: box centred radially on arc_radius, at slot bottom.
    const double sa = startRad;
    const double rx = std::cos(sa);
    const double ry = std::sin(sa);
    // Place the box so its centre lands on (cx + R*rx, cy + R*ry).
    // Box local: dx = chord (tangent ~ x), dy = slot_width, dz = slot_depth.
    const double segCx = cx + in.arc_radius_mm * rx;
    const double segCy = cy + in.arc_radius_mm * ry;
    const gp_Pnt seedOrigin(segCx - segChord / 2.0,
                            segCy - in.slot_width_mm / 2.0,
                            topZ - in.slot_depth_mm);
    const gp_Ax2 seedAx(seedOrigin, gp::DZ());
    const TopoDS_Shape seed = pr::box(
        seedAx, segChord, in.slot_width_mm, in.slot_depth_mm + kOver);

    // ── Sub-features 1..N: sweep the seed box around the arc ─────────────
    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < N; ++i) {
        const double theta = stepRad * static_cast<double>(i);
        gp_Trsf t;
        t.SetRotation(spinAxis, theta);
        BRepBuilderAPI_Transform xform(seed, t, true);
        if (!xform.IsDone())
            throw SkillError("power_reserve_indicator_slot: segment transform failed");
        current = pr::cut(current, xform.Shape());
    }

    // ── Sub-feature N+1: hand-clearance pocket (annular ring below arc) ──
    // Shallow annular relief under the arc band so the indicator hand sweeps.
    const double bandHalf = in.slot_width_mm * 1.5;
    const double ringOuter = in.arc_radius_mm + bandHalf;
    const double ringInner = std::max(0.1, in.arc_radius_mm - bandHalf);
    const double pocketDepth = std::max(0.3, in.slot_depth_mm * 0.5);
    const gp_Pnt pocketStart(cx, cy, topZ - in.slot_depth_mm - pocketDepth);
    const gp_Ax2 pocketAx(pocketStart, gp::DZ());
    const TopoDS_Shape pocketTool = pr::annularRing(
        pocketAx, ringOuter, ringInner, pocketDepth + kOver);
    current = pr::cut(current, pocketTool);

    const double vSlot = segChord * in.slot_width_mm * in.slot_depth_mm
                         * static_cast<double>(N) * 0.7;   // overlap discount
    const double vPocket = M_PI *
        (ringOuter * ringOuter - ringInner * ringInner) * pocketDepth;
    const double volRemoved = vSlot + vPocket;

    const int subCount = N + 1;

    json params = {
        { "center_xy",         { in.center_xy.X(),
                                 in.center_xy.Y(),
                                 in.center_xy.Z() } },
        { "arc_radius_mm",     in.arc_radius_mm },
        { "arc_start_deg",     in.arc_start_deg },
        { "arc_end_deg",       in.arc_end_deg },
        { "arc_segment_count", in.arc_segment_count },
        { "slot_width_mm",     in.slot_width_mm },
        { "slot_depth_mm",     in.slot_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "watch_feature_type",         "power_reserve_indicator_slot" },
        { "subfeature_count",           subCount },
        { "arc_segment_count",          in.arc_segment_count },
        { "derived_sweep_deg",          in.arc_end_deg - in.arc_start_deg },
        { "derived_arc_radius_mm",      in.arc_radius_mm },
        { "derived_ring_outer_mm",      ringOuter },
        { "derived_ring_inner_mm",      ringInner },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;end_mill";
    tooling.tool_dia_mm       = in.slot_width_mm;
    tooling.tool_length_mm    = in.slot_depth_mm + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.02;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(15.0, volRemoved / 30.0);
    tooling.extra = {
        { "watch_application", "power_reserve_indicator" },
        { "arc_segments",      in.arc_segment_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::power_reserve_indicator_slot: R={} segs={} sweep={}..{}",
                  in.arc_radius_mm, in.arc_segment_count,
                  in.arc_start_deg, in.arc_end_deg);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a swept slot leaves many planar faces along an arc;
    // a high planar-face count beyond a plain block hints at the arc slot.
    int planarCount = 0;
    for (int i = 0; i < wp.faceCount(); ++i)
        if (wp.isFacePlanar(i)) ++planarCount;
    if (planarCount >= 20) {
        json recovered = { { "arc_radius_mm",     8.0 },
                           { "arc_start_deg",      30.0 },
                           { "arc_end_deg",        150.0 },
                           { "arc_segment_count",  12 },
                           { "slot_width_mm",      1.2 },
                           { "slot_depth_mm",      0.8 } };
        json matched   = { { "source",       "geometric_arc_slot" },
                           { "planar_faces", planarCount } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::power_reserve_indicator_slot
