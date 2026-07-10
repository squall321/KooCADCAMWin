// @lat: [[engine/cam-3axis-verify#Pipeline]]
//
// Toolpath generators for slice-1 CAM (drill_hole + mill_circular_pocket).
//
// Implementation notes
// --------------------
// 1.  We *trust* the signature's `tooling` block (ToolingMeta).  Future
//     work will wire a real tool-library lookup that overrides this with
//     materials-specific cutting data; for slice-1 the skill-stamped value
//     is the source of truth.
// 2.  Feed and spindle:
//        feed_mm_per_min = cutting_speed_sfm * 12 * π / dia_mm           (chip-load formula stub)
//        spindle_rpm     = cutting_speed_sfm * 304.8 / (π * dia_mm)      (sfm→rpm)
//     These are coarse — accurate enough to anchor cycle-time estimates.
// 3.  Cycle-time = Σ(segment_length) / effective_feed, with rapids using
//     a fixed FAST = 10000 mm/min.  This matches the ±30 % accuracy noted
//     in lat.md/engine/cam-3axis-verify.md Stage-5.

#include "Toolpath.hpp"
#include "ToolLibrary.hpp"

#include <gp.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <string>

namespace koocadcam::cam {

namespace {

// ── Tunables ─────────────────────────────────────────────────────────────
constexpr double kSafeZAboveStock_mm   = 5.0;   // safe Z above bbox top
constexpr double kApproachClearance_mm = 1.0;   // rapid → clearance → feed
constexpr double kRapidFeed_mm_per_min = 10000.0;
constexpr double kHelixTurnsPerDepth   = 1.0;   // 1 full helical turn per dia of climb (coarse)
constexpr int    kHelixArcsPerTurn     = 8;     // 45° arc segments
constexpr int    kSpiralRings          = 6;     // # of concentric spiral rings filling pocket
constexpr double kAxialStepFraction    = 0.5;   // roughing depth of cut = 0.5 × tool dia per pass

// The descending cut-plane Z levels for a multi-pass (roughing) pocket: from the
// first axial step below the entry down to (and including) the full depth.  Each
// level is a fraction of the tool diameter deep; the last entry is always the
// exact floor so the finish pass lands on the true depth.  For a shallow pocket
// (depth ≤ one step) this returns just the floor — i.e. the original single pass.
std::vector<double> roughingLevels(double work_z_top, double depth, double toolD)
{
    const double step = std::max(0.2, kAxialStepFraction * (toolD > 0.0 ? toolD : depth));
    std::vector<double> zs;
    for (double d = step; d < depth - 1e-6; d += step)
        zs.push_back(work_z_top - d);
    zs.push_back(work_z_top - depth);   // final pass at the exact floor
    return zs;
}

// Compute spindle RPM from surface feet per minute.
// rpm = sfm × 12 / (π × dia_in) ; here we keep dia in mm but convert sfm→mm/min.
double sfmToRpm(double sfm, double dia_mm)
{
    if (dia_mm <= 1e-6) return 0.0;
    const double sfm_to_mm_per_min = 304.8;
    return (sfm * sfm_to_mm_per_min) / (M_PI * dia_mm);
}

// Feed = rpm × flute_count × feed_per_tooth.
double computeFeed_mm_per_min(double rpm, int flutes, double fpt_mm)
{
    if (rpm <= 0.0 || flutes <= 0 || fpt_mm <= 0.0) return 100.0;  // safe default
    return rpm * static_cast<double>(flutes) * fpt_mm;
}

// Build a synthetic tool_id label from ToolingMeta.
std::string makeToolId(const skill::ToolingMeta& tm)
{
    std::ostringstream s;
    s << tm.tool_type
      << "_" << static_cast<int>(std::round(tm.tool_dia_mm * 10.0)) << "d10mm"
      << "_" << tm.tool_material
      << "_" << tm.flute_count << "flute";
    return s.str();
}

// Euclidean distance between two points.
double dist(const gp_Pnt& a, const gp_Pnt& b)
{
    return a.Distance(b);
}

// Length of an arc segment given its previous endpoint, this endpoint, and
// the relative arc-center offset (I, J).  For helical arcs we add the dZ
// contribution (helix slope).
double arcLength(const gp_Pnt& start, const PathSegment& seg)
{
    const double cx = start.X() + seg.arc_i;
    const double cy = start.Y() + seg.arc_j;
    const double r  = std::hypot(start.X() - cx, start.Y() - cy);
    if (r < 1e-9) return 0.0;
    const double a0 = std::atan2(start.Y()       - cy, start.X()       - cx);
    const double a1 = std::atan2(seg.end_point.Y() - cy, seg.end_point.X() - cx);
    double dtheta = a1 - a0;
    // Wrap into the swept direction.
    if (seg.move == PathSegment::Move::ArcCCW) {
        while (dtheta <= 0.0) dtheta += 2.0 * M_PI;
    } else {
        while (dtheta >= 0.0) dtheta -= 2.0 * M_PI;
        dtheta = -dtheta;  // make positive arc length
    }
    const double arcXY = r * dtheta;
    const double dz    = seg.end_point.Z() - start.Z();
    return std::sqrt(arcXY * arcXY + dz * dz);
}

// Sum cycle time for a populated toolpath.  Rapids use kRapidFeed, linear/arc
// uses each segment's programmed feed (fallback to a global feed if zero).
double estimateCycleTime_s(const std::vector<PathSegment>& segs, double fallbackFeed)
{
    if (segs.empty()) return 0.0;
    double t_min = 0.0;
    gp_Pnt cur(0.0, 0.0, 0.0);
    bool initialised = false;
    for (const auto& s : segs) {
        if (!initialised) {
            // We don't know the actual start of segment 0; treat its length
            // as 0 and use end_point as the new "current" position.  This
            // under-counts the very first move slightly — acceptable for
            // ±30 % cycle-time estimates.
            cur = s.end_point;
            initialised = true;
            continue;
        }
        double len = 0.0;
        if (s.move == PathSegment::Move::ArcCW || s.move == PathSegment::Move::ArcCCW) {
            len = arcLength(cur, s);
        } else {
            len = dist(cur, s.end_point);
        }
        const double feed = (s.move == PathSegment::Move::Rapid)
                          ? kRapidFeed_mm_per_min
                          : (s.feed_mm_per_min > 0.0 ? s.feed_mm_per_min : fallbackFeed);
        if (feed > 1e-6) t_min += len / feed;
        cur = s.end_point;
    }
    return t_min * 60.0;
}

// Entry-plane Z (work offset) for a feature.  Slice-1 generators assumed the
// entry face sits at Z=0, but recovered features carry their true axial entry
// (bore/countersink/counterbore emit `position_z_mm` as the full 3-D entry
// point; rect-pocket/slot a `center_z_mm`/`start_z_mm`).  Honouring it keeps
// the toolpath on the real surface when the feature is not at Z=0 — otherwise
// every re-synthesised part is machined as if its top were the origin.
double entryZ(const nlohmann::json& params, const char* key = "position_z_mm")
{
    return params.value(key, 0.0);
}

// The 3-axis-Z post can only machine a feature whose cut axis is global ±Z.  A
// recovered RADIAL feature (side bore/pocket/button — axis along ±X/±Y) needs a
// re-setup or a 4th axis the post does not have; routing it through a Z generator
// would emit a plausible-but-WRONG path (a vertical plunge at (x,y) instead of
// into the side face).  So detect a non-Z cut axis and refuse.  The axis lives in
// `axis_dir` (drill/bore/pocket) or `face_normal` (box_pocket/box_boss).
bool cutAxisIsZ(const nlohmann::json& params)
{
    for (const char* key : { "axis_dir", "face_normal", "axis_dir_xyz" }) {
        if (params.contains(key) && params[key].is_array() && params[key].size() == 3) {
            const auto& a = params[key];
            const double z = a[2].get<double>();
            const double x = a[0].get<double>(), y = a[1].get<double>();
            const double n = std::sqrt(x * x + y * y + z * z);
            if (n > 1e-9) return std::abs(z / n) > 0.999;   // ~parallel to ±Z
        }
    }
    return true;   // no axis authored → the legacy ±Z assumption
}

// An empty toolpath for a feature the 3-axis-Z post cannot machine, with a loud
// warning — so a radial feature FAILS VISIBLY instead of shipping a wrong path.
Toolpath emptyRadialToolpath(const skill::FeatureSignature& sig)
{
    spdlog::warn("cam: '{}' has a non-Z (radial/side) cut axis — not machinable on "
                 "the 3-axis-Z post (needs a re-setup or a 4th axis); emitting an "
                 "empty toolpath", sig.skill_id);
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    return tp;
}

// Read the cut axis (into the material) from the signature.  drill/bore emit
// `axis_dir`; if absent, default -Z (legacy downward drill).
gp_Dir cutAxisDir(const nlohmann::json& params)
{
    for (const char* key : { "axis_dir", "axis_dir_xyz" }) {
        if (params.contains(key) && params[key].is_array() && params[key].size() == 3) {
            const auto& a = params[key];
            const double x = a[0].get<double>(), y = a[1].get<double>(), z = a[2].get<double>();
            if (std::sqrt(x * x + y * y + z * z) > 1e-9) return gp_Dir(x, y, z);
        }
    }
    return gp_Dir(0.0, 0.0, -1.0);
}

// Read a 3-vector param (e.g. face_normal, face_xaxis) as a gp_Vec, or a zero
// vector when the key is missing/degenerate (caller falls back).
gp_Vec readVec3(const nlohmann::json& params, const char* key)
{
    if (params.contains(key) && params[key].is_array() && params[key].size() == 3) {
        const auto& a = params[key];
        return gp_Vec(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
    }
    return gp_Vec(0.0, 0.0, 0.0);
}

// Build an orthonormal in-plane basis (u, v) perpendicular to `depthAxis`.  When
// `preferU` is non-degenerate and not parallel to the axis it seeds u (so a
// pocket's length direction lands on u); otherwise DX (or DY) is projected off
// the axis.  Returns a right-handed (u, v, depthAxis) frame.
void buildInPlaneBasis(const gp_Dir& depthAxis, const gp_Vec& preferU,
                       gp_Dir& u_out, gp_Dir& v_out)
{
    const gp_Vec dv(depthAxis);
    gp_Vec u = preferU;
    if (u.Magnitude() < 1e-9 || std::abs(dv.Dot(gp_Vec(u.Normalized()))) > 0.999)
        u = (std::abs(dv.Dot(gp_Vec(gp::DX()))) > 0.99) ? gp_Vec(gp::DY()) : gp_Vec(gp::DX());
    u = u - dv * dv.Dot(u);            // project into the plane ⊥ depthAxis
    u.Normalize();
    u_out = gp_Dir(u);
    v_out = gp_Dir(dv.Crossed(u));     // right-handed
}

// Emit N coaxial plunges in a feature's LOCAL work-plane frame: build (u, v)
// from the depth axis, record the frame on `tp`, then for each depth d plunge
// from above the surface (negative local depth) down to +d (into the material),
// u = v = 0 (a point plunge on the axis).  Shared by the radial drill / bore /
// counterbore / countersink / ream generators — a radial hole family feature is
// just one or more coaxial axial plunges in its own frame.
void emitCoaxialPlungesInLocalFrame(Toolpath& tp, const gp_Pnt& origin,
                                    const gp_Dir& depthAxis,
                                    const std::vector<double>& depths, double feed)
{
    gp_Dir u, v;
    buildInPlaneBasis(depthAxis, gp_Vec(0, 0, 0), u, v);
    tp.work_origin     = origin;
    tp.work_u_axis     = u;
    tp.work_v_axis     = v;
    tp.work_depth_axis = depthAxis;

    const double safeLocal = -kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    for (double d : depths) {
        tp.segments.push_back({ M::Rapid,  gp_Pnt(0.0, 0.0, safeLocal),               0.0,  0, 0, 0 });
        tp.segments.push_back({ M::Rapid,  gp_Pnt(0.0, 0.0, -kApproachClearance_mm),  0.0,  0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(0.0, 0.0, d),                       feed, 0, 0, 0 });
        tp.segments.push_back({ M::Rapid,  gp_Pnt(0.0, 0.0, safeLocal),               0.0,  0, 0, 0 });
    }
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
}

}  // namespace

// ── drill_hole toolpath ──────────────────────────────────────────────────

Toolpath drillHoleToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);
    tp.coolant_pressure = 0.0;

    // Pull positional params from the signature.
    const double x      = sig.params.value("position_x_mm", 0.0);
    const double y      = sig.params.value("position_y_mm", 0.0);
    const double depth  = sig.params.value("depth_mm", 0.0);
    const bool   thru   = sig.params.value("through_hole", false);
    const double dia    = sig.params.value("diameter_mm",
                          sig.tooling.tool_dia_mm > 0 ? sig.tooling.tool_dia_mm : 1.0);

    // The skill convention: axis_dir = (0, 0, -1) drilling DOWN.  We assume
    // the entry plane Z is provided implicitly (we use 0 if unavailable).
    // For the slice-1 generator we work in WORKPIECE coords where Z=0 is
    // the entry face top, and depth is taken along -Z.  This matches the
    // drill_hole skill's `position_*_mm` convention when the entry face is
    // the top of a cuboid stock.  When the feature carries its own axial entry
    // (recovered bore/countersink/counterbore), honour it instead of Z=0.
    const double work_z_top = entryZ(sig.params);   // entry face Z (work offset)
    const double safe_z     = work_z_top + kSafeZAboveStock_mm;
    const double cut_z      = work_z_top - (thru ? depth + 2.0 : depth);

    const double rpm  = tp.spindle_rpm;
    const double feed = computeFeed_mm_per_min(rpm, sig.tooling.flute_count,
                                               sig.tooling.feed_per_tooth_mm);
    (void)dia;

    // 4 segments.
    tp.segments.reserve(4);
    // [0] Rapid to (x, y, safe_z) — assume tool is somewhere above; this
    //      is the unambiguous "go to safety" move.
    tp.segments.push_back({PathSegment::Move::Rapid,
                           gp_Pnt(x, y, safe_z), 0.0, 0, 0, 0});
    // [1] Rapid to approach clearance just above the work surface.
    tp.segments.push_back({PathSegment::Move::Rapid,
                           gp_Pnt(x, y, work_z_top + kApproachClearance_mm),
                           0.0, 0, 0, 0});
    // [2] Plunge: linear feed to depth.
    tp.segments.push_back({PathSegment::Move::Linear,
                           gp_Pnt(x, y, cut_z), feed, 0, 0, 0});
    // [3] Retract back to safe_z.
    tp.segments.push_back({PathSegment::Move::Rapid,
                           gp_Pnt(x, y, safe_z), 0.0, 0, 0, 0});

    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── radial drill / bore toolpath (local-frame) ────────────────────────────
// A side bore / side drill whose axis is NOT global ±Z.  A 3-axis-Z post cannot
// reach it in the global frame, so we emit the plunge in the feature's OWN local
// frame: origin = the entry point, depth axis = the cut axis, and the path is a
// simple axial plunge in local (u, v, depth) — (0,0,-safe) → (0,0,+depth) →
// retract.  The Toolpath records the work-plane frame so a re-setup / post can
// rotate the local coords to world.  (This is the honest model: the geometry is
// correct in its own frame, and the frame is carried for the post — no lie about
// the part being machinable as-is on a pure 3-axis-Z setup.)
Toolpath radialDrillToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double px    = sig.params.value("position_x_mm", 0.0);
    const double py    = sig.params.value("position_y_mm", 0.0);
    const double pz    = sig.params.value("position_z_mm", 0.0);
    const double depth = sig.params.value("depth_mm", 0.0);
    const bool   thru  = sig.params.value("through_hole", false);
    const double feed  = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                sig.tooling.feed_per_tooth_mm);

    const gp_Dir depthAxis = cutAxisDir(sig.params);   // into the material
    // Build an orthonormal in-plane basis (u, v) perpendicular to the depth axis.
    gp_Vec u(gp::DX());
    if (std::abs(gp_Vec(depthAxis).Dot(u)) > 0.99) u = gp_Vec(gp::DY());
    u = u - gp_Vec(depthAxis) * gp_Vec(depthAxis).Dot(u);
    u.Normalize();
    const gp_Vec v = gp_Vec(depthAxis).Crossed(u);

    // Record the work-plane frame (origin = the entry point on the side face).
    tp.work_origin     = gp_Pnt(px, py, pz);
    tp.work_u_axis     = gp_Dir(u);
    tp.work_v_axis     = gp_Dir(v);
    tp.work_depth_axis = depthAxis;

    // Emit the plunge in LOCAL coords: depth runs along +Z_local (into the
    // material), u/v = 0 (a point plunge).  safe/approach are above the surface
    // (negative local depth).
    const double localCut  = thru ? depth + 2.0 : depth;
    const double safeLocal = -kSafeZAboveStock_mm;   // above the entry surface
    using M = PathSegment::Move;
    tp.segments.push_back({ M::Rapid,  gp_Pnt(0.0, 0.0, safeLocal),               0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Rapid,  gp_Pnt(0.0, 0.0, -kApproachClearance_mm),  0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(0.0, 0.0, localCut),                feed, 0, 0, 0 });
    tp.segments.push_back({ M::Rapid,  gp_Pnt(0.0, 0.0, safeLocal),               0.0,  0, 0, 0 });

    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── radial box_pocket toolpath (local-frame) ──────────────────────────────
// A side button / side recess whose face_normal is NOT ±Z.  Emits the same
// inward-offset rectangular finishing contour as the +Z box_pocket, but in the
// feature's LOCAL (u, v, depth) frame: origin = the world mouth centre, u = the
// in-plane length axis (face_xaxis), v = normal × u, depth = -face_normal (into
// the material).  The Toolpath records the frame for a re-setup / post.
Toolpath radialBoxPocketToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double ox    = sig.params.value("center_x_world_mm", 0.0);
    const double oy    = sig.params.value("center_y_world_mm", 0.0);
    const double oz    = sig.params.value("center_z_world_mm", 0.0);
    const double L     = sig.params.value("length_mm", 10.0);
    const double W     = sig.params.value("width_mm", 10.0);
    const double depth = sig.params.value("depth_mm", 1.0);
    const double toolD = (sig.tooling.tool_dia_mm > 0.0)
                       ? sig.tooling.tool_dia_mm : std::min(L, W) * 0.4;
    const double toolR = toolD * 0.5;
    const double feed  = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                sig.tooling.feed_per_tooth_mm);

    // depth axis = -face_normal (the outward normal points OUT of the face; the
    // recess sinks in).  u = the in-plane length axis; v completes the frame.
    const gp_Vec normal = readVec3(sig.params, "face_normal");
    const gp_Dir depthAxis = (normal.Magnitude() > 1e-9)
                           ? gp_Dir(-normal.X(), -normal.Y(), -normal.Z())
                           : cutAxisDir(sig.params);
    gp_Dir u, v;
    buildInPlaneBasis(depthAxis, readVec3(sig.params, "face_xaxis"), u, v);

    tp.work_origin     = gp_Pnt(ox, oy, oz);
    tp.work_u_axis     = u;
    tp.work_v_axis     = v;
    tp.work_depth_axis = depthAxis;

    // Inward-offset rectangle in LOCAL (u, v), at each descending depth level.
    const double hu = std::max(0.1, L / 2.0 - toolR);
    const double hv = std::max(0.1, W / 2.0 - toolR);
    const double safeLocal = -kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    tp.segments.push_back({ M::Rapid, gp_Pnt(-hu, -hv, safeLocal), 0.0, 0, 0, 0 });
    for (double dLevel : roughingLevels(0.0, depth, toolD)) {
        const double z = -dLevel;   // roughingLevels returns work_z_top - d = -d here (work_z_top=0)
        tp.segments.push_back({ M::Linear, gp_Pnt(-hu, -hv, z), feed, 0, 0, 0 }); // plunge/step
        tp.segments.push_back({ M::Linear, gp_Pnt( hu, -hv, z), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt( hu,  hv, z), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(-hu,  hv, z), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(-hu, -hv, z), feed, 0, 0, 0 }); // close
    }
    tp.segments.push_back({ M::Rapid, gp_Pnt(-hu, -hv, safeLocal), 0.0, 0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── radial box_boss toolpath (local-frame) ────────────────────────────────
// A raised rectangular island on a SIDE face (a lug on a watch-case side).  The
// +Z boxBossToolpath clears the surrounding field with an OUTWARD-offset rectangle
// at the base plane; the radial version does the same in the feature's LOCAL frame
// — origin = world base centre, u = face_xaxis (length), v = normal × u, depth =
// +face_normal (the boss stands OUT along +normal, so we clear the field just
// BELOW the base = at local depth 0, skirting the boss walls by +toolR).
Toolpath radialBoxBossToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double ox    = sig.params.value("center_x_world_mm", 0.0);
    const double oy    = sig.params.value("center_y_world_mm", 0.0);
    const double oz    = sig.params.value("center_z_world_mm", 0.0);
    const double L     = sig.params.value("length_mm", 10.0);
    const double W     = sig.params.value("width_mm", 10.0);
    const double toolD = (sig.tooling.tool_dia_mm > 0.0)
                       ? sig.tooling.tool_dia_mm : std::min(L, W) * 0.4;
    const double toolR = toolD * 0.5;
    const double feed  = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                sig.tooling.feed_per_tooth_mm);

    // face_normal points OUT of the boss cap; the field-clearing pass runs at the
    // base plane, so the local depth axis is -face_normal (into the host body).
    const gp_Vec normal = readVec3(sig.params, "face_normal");
    const gp_Dir depthAxis = (normal.Magnitude() > 1e-9)
                           ? gp_Dir(-normal.X(), -normal.Y(), -normal.Z())
                           : cutAxisDir(sig.params);
    gp_Dir u, v;
    buildInPlaneBasis(depthAxis, readVec3(sig.params, "face_xaxis"), u, v);

    tp.work_origin     = gp_Pnt(ox, oy, oz);
    tp.work_u_axis     = u;
    tp.work_v_axis     = v;
    tp.work_depth_axis = depthAxis;

    // OUTWARD-offset rectangle in LOCAL (u, v) at the base plane (local depth 0) —
    // the tool skirts the boss wall by +toolR, clearing the adjacent field.
    const double hu = L / 2.0 + toolR;
    const double hv = W / 2.0 + toolR;
    const double safeLocal = -kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    tp.segments.push_back({ M::Rapid,  gp_Pnt(-hu, -hv, safeLocal), 0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(-hu, -hv, 0.0),       feed, 0, 0, 0 }); // plunge to base
    tp.segments.push_back({ M::Linear, gp_Pnt( hu, -hv, 0.0),       feed, 0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt( hu,  hv, 0.0),       feed, 0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(-hu,  hv, 0.0),       feed, 0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(-hu, -hv, 0.0),       feed, 0, 0, 0 }); // close
    tp.segments.push_back({ M::Rapid,  gp_Pnt(-hu, -hv, safeLocal), 0.0,  0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── radial dome_boss toolpath (local-frame) ───────────────────────────────
// A raised spherical-cap island on a SIDE face.  The +Z domeBossToolpath finishes
// the cap with a ball-nose Z-level ring stack (ball CENTRE on a concentric sphere
// Rs+toolR) + a base skirt.  The radial version emits the SAME rings in the
// feature's LOCAL frame: local depth measures OUTWARD along +normal from the base
// plane (the cap protrudes), so a contact at surface height h_above_base sits at
// local depth = -(that height) below the tool, and the ball centre lifts by the
// concentric-sphere scale.  We emit in local coords where local Z is along the cap
// axis (+normal), rings ascending from the base to the apex.
Toolpath radialDomeBossToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double ox     = sig.params.value("center_x_world_mm", 0.0);
    const double oy     = sig.params.value("center_y_world_mm", 0.0);
    const double oz     = sig.params.value("center_z_world_mm", 0.0);
    const double baseR  = sig.params.value("base_radius_mm", 5.0);
    const double height = sig.params.value("height_mm", 1.0);
    const double toolD  = (sig.tooling.tool_dia_mm > 0.0)
                        ? sig.tooling.tool_dia_mm : baseR * 0.4;
    const double toolR  = toolD * 0.5;
    const double feed   = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                 sig.tooling.feed_per_tooth_mm);

    // Local frame: depth axis = +face_normal (the cap protrudes OUT along +normal,
    // so a point h above the base is at local +h along the cap axis).  u/v span the
    // base plane.  origin = the world base centre.
    const gp_Vec normal = readVec3(sig.params, "face_normal");
    const gp_Dir capAxis = (normal.Magnitude() > 1e-9)
                         ? gp_Dir(normal.X(), normal.Y(), normal.Z())
                         : gp_Dir(0, 0, 1);
    gp_Dir u, v;
    buildInPlaneBasis(capAxis, gp_Vec(0, 0, 0), u, v);
    tp.work_origin     = gp_Pnt(ox, oy, oz);
    tp.work_u_axis     = u;
    tp.work_v_axis     = v;
    tp.work_depth_axis = capAxis;

    // Ball-nose ring stack in LOCAL coords: local z = height above the base along
    // the cap axis (apex at local z = height).  Same concentric-sphere ball-centre
    // scale as the +Z dome (Rs+toolR)/Rs about the sphere centre at local z = height
    // - Rs.  (Note: in the local frame "up the cap" is +z, matching the +Z dome's
    // world +Z convention with baseZ = 0.)
    const double Rs    = (height > 1e-6) ? (baseR * baseR + height * height) / (2.0 * height)
                                         : baseR;
    const double scale = (Rs > 1e-6) ? (Rs + toolR) / Rs : 1.0;
    const double sphereCenterZ = height - Rs;   // local, base plane at z=0
    const double safeLocal = height + kSafeZAboveStock_mm;

    using M = PathSegment::Move;
    auto emitLocalRing = [&](double r, double z) {
        const double quarter[4][2] = { {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}, {1.0, 0.0} };
        gp_Pnt cur(r, 0.0, z);
        for (int q = 0; q < 4; ++q) {
            const double nu = r * quarter[q][0];
            const double nv = r * quarter[q][1];
            const double ic = 0.0 - cur.X();
            const double jc = 0.0 - cur.Y();
            tp.segments.push_back({ M::ArcCCW, gp_Pnt(nu, nv, z), feed, ic, jc, 0.0 });
            cur = gp_Pnt(nu, nv, z);
        }
    };

    const double stepover = std::max(0.2, toolD * 0.25);
    const int    nLevels  = std::max(1, static_cast<int>(std::ceil(height / stepover)));
    bool positioned = false;
    for (int k = 1; k <= nLevels; ++k) {
        const double zc  = height - (height * static_cast<double>(k) / static_cast<double>(nLevels)); // contact z (local)
        const double d   = height - zc;                       // drop below apex
        const double rho = std::sqrt(std::max(0.0, 2.0 * Rs * d - d * d));
        const double r   = std::max(0.1, rho * scale);
        const double zb  = std::max(0.0, sphereCenterZ + (zc - sphereCenterZ) * scale);
        if (!positioned) {
            tp.segments.push_back({ M::Rapid,  gp_Pnt(r, 0.0, safeLocal), 0.0,  0, 0, 0 });
            tp.segments.push_back({ M::Linear, gp_Pnt(r, 0.0, zb),        feed, 0, 0, 0 });
            positioned = true;
        } else {
            tp.segments.push_back({ M::Linear, gp_Pnt(r, 0.0, zb), feed, 0, 0, 0 });
        }
        emitLocalRing(r, zb);
    }
    // Base skirt at the base plane (local z=0), offset OUTWARD by the tool radius.
    const double skirtR = baseR + toolR;
    tp.segments.push_back({ M::Linear, gp_Pnt(skirtR, 0.0, 0.0), feed, 0, 0, 0 });
    emitLocalRing(skirtR, 0.0);
    tp.segments.push_back({ M::Rapid, gp_Pnt(skirtR, 0.0, safeLocal), 0.0, 0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── radial mill_circular_pocket toolpath (local-frame) ────────────────────
// A round side pocket / side recess whose axis is NOT ±Z.  Emits an inward
// circular finishing contour (radius = diameter/2 - toolR) in the feature's LOCAL
// (u, v) plane at each descending depth level — the circular analogue of
// radialBoxPocketToolpath's rectangle.  origin = the world entry, depth = axis_dir.
Toolpath radialCircularPocketToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double px    = sig.params.value("position_x_mm", 0.0);
    const double py    = sig.params.value("position_y_mm", 0.0);
    const double pz    = sig.params.value("position_z_mm", 0.0);
    const double diam  = sig.params.value("diameter_mm", 10.0);
    const double depth = sig.params.value("depth_mm", 1.0);
    const double toolD = (sig.tooling.tool_dia_mm > 0.0 && sig.tooling.tool_dia_mm < diam)
                       ? sig.tooling.tool_dia_mm : diam * 0.5;
    const double toolR = toolD * 0.5;
    const double feed  = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                sig.tooling.feed_per_tooth_mm);

    const gp_Dir depthAxis = cutAxisDir(sig.params);
    gp_Dir u, v;
    buildInPlaneBasis(depthAxis, gp_Vec(0, 0, 0), u, v);
    tp.work_origin     = gp_Pnt(px, py, pz);
    tp.work_u_axis     = u;
    tp.work_v_axis     = v;
    tp.work_depth_axis = depthAxis;

    const double r = std::max(0.1, diam / 2.0 - toolR);   // inward finishing radius
    const double safeLocal = -kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    tp.segments.push_back({ M::Rapid, gp_Pnt(r, 0.0, safeLocal), 0.0, 0, 0, 0 });
    // roughingLevels(0, depth, toolD) → 0 - d = -d; the local cut depth is +d.
    const double quarter[4][2] = { {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}, {1.0, 0.0} };
    for (double dLevel : roughingLevels(0.0, depth, toolD)) {
        const double z = -dLevel;   // +depth into the material
        tp.segments.push_back({ M::Linear, gp_Pnt(r, 0.0, z), feed, 0, 0, 0 }); // plunge/step to this level
        gp_Pnt cur(r, 0.0, z);
        for (int q = 0; q < 4; ++q) {
            const double nu = r * quarter[q][0];
            const double nv = r * quarter[q][1];
            const double ic = 0.0 - cur.X();   // centre (local origin) - start
            const double jc = 0.0 - cur.Y();
            tp.segments.push_back({ M::ArcCCW, gp_Pnt(nu, nv, z), feed, ic, jc, 0.0 });
            cur = gp_Pnt(nu, nv, z);
        }
    }
    tp.segments.push_back({ M::Rapid, gp_Pnt(r, 0.0, safeLocal), 0.0, 0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── radial mill_slot toolpath (local-frame) ───────────────────────────────
// A side slot / side port (a USB-C / SIM-tray cutout) whose cut axis is NOT ±Z.
// The +Z millSlotToolpath is a centreline traverse (start → end at depth); the
// radial version does the same in the feature's LOCAL frame: origin = the world
// start point, depth = axis_dir (into the side face), u = the slot centreline
// direction (end − start, which lies in the entry plane), v = depth × u.  The
// centreline runs along +u at local depth, from local 0 to the slot length.
Toolpath radialSlotToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const gp_Pnt start(sig.params.value("start_x_mm", 0.0),
                       sig.params.value("start_y_mm", 0.0),
                       sig.params.value("start_z_mm", 0.0));
    const gp_Pnt end  (sig.params.value("end_x_mm", 0.0),
                       sig.params.value("end_y_mm", 0.0),
                       sig.params.value("end_z_mm", 0.0));
    const double depth = sig.params.value("depth_mm", 1.0);
    const double feed  = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                sig.tooling.feed_per_tooth_mm);

    const gp_Dir depthAxis = cutAxisDir(sig.params);   // into the material
    // Prefer the actual slot centreline (end − start) as the local u; buildInPlane
    // projects it off the axis (it already lies in the entry plane) and normalises.
    const gp_Vec span(start, end);
    gp_Dir u, v;
    buildInPlaneBasis(depthAxis, span, u, v);
    tp.work_origin     = start;
    tp.work_u_axis     = u;
    tp.work_v_axis     = v;
    tp.work_depth_axis = depthAxis;

    // Slot length = the centreline span (v ≈ 0 since span ∥ u after projection).
    const double slotLen = span.Magnitude();
    const double safeLocal = -kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    tp.segments.push_back({ M::Rapid,  gp_Pnt(0.0,     0.0, safeLocal), 0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(0.0,     0.0, depth),     feed, 0, 0, 0 }); // plunge
    tp.segments.push_back({ M::Linear, gp_Pnt(slotLen, 0.0, depth),     feed, 0, 0, 0 }); // traverse
    tp.segments.push_back({ M::Rapid,  gp_Pnt(slotLen, 0.0, safeLocal), 0.0,  0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── radial hole-pattern toolpath (local-frame) ────────────────────────────
// A side grille / side bolt-circle whose axis is NOT ±Z.  Emits a per-hole plunge
// like holePatternToolpath, but in the pattern's LOCAL (u, v, depth) frame: each
// recovered world hole centre is projected onto (u, v) about the frame origin
// (the first hole centre), and plunged along +depth.
Toolpath radialHolePatternToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double depth = sig.params.value("hole_depth_mm", 0.0);
    const double feed  = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                sig.tooling.feed_per_tooth_mm);

    if (!sig.params.contains("hole_centers") || !sig.params["hole_centers"].is_array()
        || sig.params["hole_centers"].empty()) {
        spdlog::warn("cam: radial '{}' has no hole_centers — emitting empty toolpath",
                     sig.skill_id);
        return tp;
    }

    // depth axis = the cut axis (into the material); for a pattern face_normal is
    // the outward face dir, so prefer axis_dir/axis_dir_xyz, else -face_normal.
    gp_Dir depthAxis = cutAxisDir(sig.params);
    if (!sig.params.contains("axis_dir") && !sig.params.contains("axis_dir_xyz")) {
        const gp_Vec n = readVec3(sig.params, "face_normal");
        if (n.Magnitude() > 1e-9) depthAxis = gp_Dir(-n.X(), -n.Y(), -n.Z());
    }
    gp_Dir u, v;
    buildInPlaneBasis(depthAxis, gp_Vec(0, 0, 0), u, v);

    // Origin = the first hole centre; project every hole onto (u, v) about it.
    const auto& centers = sig.params["hole_centers"];
    const auto& c0 = centers[0];
    const gp_Pnt origin(c0[0].get<double>(), c0[1].get<double>(),
                        (c0.size() >= 3) ? c0[2].get<double>() : 0.0);
    tp.work_origin     = origin;
    tp.work_u_axis     = u;
    tp.work_v_axis     = v;
    tp.work_depth_axis = depthAxis;

    const gp_Vec uv(u), vv(v);
    const double safeLocal = -kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    for (const auto& hc : centers) {
        if (!hc.is_array() || hc.size() < 2) continue;
        const gp_Pnt hp(hc[0].get<double>(), hc[1].get<double>(),
                        (hc.size() >= 3) ? hc[2].get<double>() : 0.0);
        const gp_Vec rel(origin, hp);
        const double uOff = rel.Dot(uv);
        const double vOff = rel.Dot(vv);
        tp.segments.push_back({ M::Rapid,  gp_Pnt(uOff, vOff, safeLocal), 0.0,  0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(uOff, vOff, depth),     feed, 0, 0, 0 });
        tp.segments.push_back({ M::Rapid,  gp_Pnt(uOff, vOff, safeLocal), 0.0,  0, 0, 0 });
    }
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── mill_circular_pocket toolpath ────────────────────────────────────────

Toolpath millCircularPocketToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);
    tp.coolant_pressure = 0.0;

    // A radial (side-face) circular pocket = an inward circular contour in the
    // feature's local work-plane frame (the circular analogue of the side box
    // pocket).  Route it before the +Z helix/spiral body.
    if (!cutAxisIsZ(sig.params)) return radialCircularPocketToolpath(sig);

    const double x       = sig.params.value("position_x_mm", 0.0);
    const double y       = sig.params.value("position_y_mm", 0.0);
    const double depth   = sig.params.value("depth_mm", 1.0);
    const double pocketD = sig.params.value("diameter_mm", 10.0);
    // Choose a tool diameter slightly smaller than the pocket; if tooling
    // metadata wasn't filled in (some specialised skills), fall back to
    // 50 % of the pocket diameter.
    const double toolD   = (sig.tooling.tool_dia_mm > 0.0 && sig.tooling.tool_dia_mm < pocketD)
                         ? sig.tooling.tool_dia_mm
                         : pocketD * 0.5;
    const double toolR   = toolD * 0.5;
    const double pocketR = pocketD * 0.5;

    const double work_z_top = entryZ(sig.params);   // entry face Z (work offset)
    const double safe_z     = work_z_top + kSafeZAboveStock_mm;

    const double rpm  = tp.spindle_rpm;
    const double feed = computeFeed_mm_per_min(rpm, sig.tooling.flute_count,
                                               sig.tooling.feed_per_tooth_mm);

    // Helical entry uses a small offset radius (~tool radius / 2) so the
    // tool descends inside the pocket bounds.
    const double helixR    = std::min(toolR * 0.5, pocketR - toolR - 0.05);
    const double helixR_eff = std::max(helixR, 0.1);

    // Approach moves.
    tp.segments.reserve(4 + kHelixArcsPerTurn * 2 + kSpiralRings * kHelixArcsPerTurn + 2);
    tp.segments.push_back({PathSegment::Move::Rapid,
                           gp_Pnt(x, y, safe_z), 0.0, 0, 0, 0});
    // Move to helix start point: (x + helixR, y) at clearance.
    tp.segments.push_back({PathSegment::Move::Rapid,
                           gp_Pnt(x + helixR_eff, y, work_z_top + kApproachClearance_mm),
                           0.0, 0, 0, 0});
    // Plunge to entry plane.
    tp.segments.push_back({PathSegment::Move::Linear,
                           gp_Pnt(x + helixR_eff, y, work_z_top),
                           feed, 0, 0, 0});

    // ── Helical entry ────────────────────────────────────────────────
    //
    // We descend in N small arc segments forming a coarse helix.  Z step
    // per arc = depth / (arcsPerTurn * turns).  Each arc is a 1/arcsPerTurn
    // share of a full ArcCCW.  We use ArcCCW (climb milling convention).
    const int    turns       = std::max(1, static_cast<int>(std::round(kHelixTurnsPerDepth * depth / std::max(0.1, toolD))));
    const int    helixSegs   = turns * kHelixArcsPerTurn;
    const double dZ_per_seg  = -depth / static_cast<double>(helixSegs);
    const double dTheta      = 2.0 * M_PI / static_cast<double>(kHelixArcsPerTurn);

    // Track the running position to compute (I, J) center offsets.
    gp_Pnt cur(x + helixR_eff, y, work_z_top);
    for (int i = 0; i < helixSegs; ++i) {
        const double theta1 = dTheta * static_cast<double>(i + 1);
        // I, J = center - start (relative).  Center is at (x, y).
        const double ic = x - cur.X();
        const double jc = y - cur.Y();
        const double nx = x + helixR_eff * std::cos(theta1);
        const double ny = y + helixR_eff * std::sin(theta1);
        const double nz = cur.Z() + dZ_per_seg;
        tp.segments.push_back({PathSegment::Move::ArcCCW,
                               gp_Pnt(nx, ny, nz), feed, ic, jc, 0.0});
        cur = gp_Pnt(nx, ny, nz);
    }

    // After the helix, we're at depth on the helix radius.  Close the
    // bottom circle (full ArcCCW back to the same angle, but already at
    // depth).  Skipped — the spiral-out below handles the rest.

    // ── Spiral-out finishing ─────────────────────────────────────────
    //
    // Walk outwards in kSpiralRings concentric rings, each ring composed of
    // kHelixArcsPerTurn ArcCCW segments.  Each ring radius = previous + dR.
    const double startR = helixR_eff;
    const double endR   = std::max(startR + 0.1, pocketR - toolR);
    const double dR     = (endR - startR) / static_cast<double>(kSpiralRings);

    for (int ring = 1; ring <= kSpiralRings; ++ring) {
        const double r = startR + dR * static_cast<double>(ring);
        // For a spiral, each step from a point on the previous ring to a
        // point on this ring at a slightly larger radius is rendered as an
        // ArcCCW that approximately traces 1/kHelixArcsPerTurn of a turn.
        for (int k = 0; k < kHelixArcsPerTurn; ++k) {
            const double theta = dTheta * static_cast<double>(k + 1);
            // Blend radius from (r - dR/arcs * (k+1)/arcs) → r — approximate
            // by stepping radius monotonically per sub-segment.
            const double rNow = r - dR * (1.0 - static_cast<double>(k + 1)
                                            / static_cast<double>(kHelixArcsPerTurn));
            const double ic = x - cur.X();
            const double jc = y - cur.Y();
            const double nx = x + rNow * std::cos(theta);
            const double ny = y + rNow * std::sin(theta);
            tp.segments.push_back({PathSegment::Move::ArcCCW,
                                   gp_Pnt(nx, ny, cur.Z()), feed, ic, jc, 0.0});
            cur = gp_Pnt(nx, ny, cur.Z());
        }
    }

    // Retract.
    tp.segments.push_back({PathSegment::Move::Rapid,
                           gp_Pnt(cur.X(), cur.Y(), safe_z), 0.0, 0, 0, 0});

    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── mill_rect_pocket toolpath ─────────────────────────────────────────────
// A perimeter contour offset inward by the tool radius, cut in MULTIPLE axial
// passes (roughing): a plunge on the first level then a contour at each
// descending Z, ending with a finish pass at the exact floor.  A shallow pocket
// (depth ≤ one axial step) collapses to the original single pass.
Toolpath millRectPocketToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double cx    = sig.params.value("center_x_mm", 0.0);
    const double cy    = sig.params.value("center_y_mm", 0.0);
    const double L     = sig.params.value("length_mm", 10.0);
    const double W     = sig.params.value("width_mm", 10.0);
    const double depth = sig.params.value("depth_mm", 1.0);
    const double toolD = (sig.tooling.tool_dia_mm > 0.0)
                       ? sig.tooling.tool_dia_mm : std::min(L, W) * 0.4;
    const double toolR = toolD * 0.5;

    const double work_z_top = entryZ(sig.params, "center_z_mm");  // entry face Z
    const double safe_z = work_z_top + kSafeZAboveStock_mm;
    const double feed   = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                 sig.tooling.feed_per_tooth_mm);
    const double hx = std::max(0.1, L / 2.0 - toolR);
    const double hy = std::max(0.1, W / 2.0 - toolR);

    using M = PathSegment::Move;
    tp.segments.push_back({ M::Rapid, gp_Pnt(cx - hx, cy - hy, safe_z), 0.0, 0, 0, 0 });
    for (double zc : roughingLevels(work_z_top, depth, toolD)) {
        // Plunge (first level) / step down (subsequent) then trace the perimeter.
        tp.segments.push_back({ M::Linear, gp_Pnt(cx - hx, cy - hy, zc), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(cx + hx, cy - hy, zc), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(cx + hx, cy + hy, zc), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(cx - hx, cy + hy, zc), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(cx - hx, cy - hy, zc), feed, 0, 0, 0 }); // close
    }
    tp.segments.push_back({ M::Rapid, gp_Pnt(cx - hx, cy - hy, safe_z), 0.0, 0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── mill_slot toolpath ────────────────────────────────────────────────────
// Traverse the slot centreline (start -> end) at depth — the slot is defined by
// its two end points + width, so the centreline IS the cutting path.
Toolpath millSlotToolpath(const skill::FeatureSignature& sig)
{
    // A radial (side-face) slot / side port is machined in its own local frame.
    if (!cutAxisIsZ(sig.params)) return radialSlotToolpath(sig);

    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double sx    = sig.params.value("start_x_mm", 0.0);
    const double sy    = sig.params.value("start_y_mm", 0.0);
    const double ex    = sig.params.value("end_x_mm", 10.0);
    const double ey    = sig.params.value("end_y_mm", 0.0);
    const double depth = sig.params.value("depth_mm", 1.0);

    const double work_z_top = entryZ(sig.params, "start_z_mm");   // entry face Z
    const double safe_z = work_z_top + kSafeZAboveStock_mm;
    const double feed   = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                 sig.tooling.feed_per_tooth_mm);
    const double zc = work_z_top - depth;

    using M = PathSegment::Move;
    tp.segments.push_back({ M::Rapid,  gp_Pnt(sx, sy, safe_z), 0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(sx, sy, zc),     feed, 0, 0, 0 }); // plunge
    tp.segments.push_back({ M::Linear, gp_Pnt(ex, ey, zc),     feed, 0, 0, 0 }); // traverse
    tp.segments.push_back({ M::Rapid,  gp_Pnt(ex, ey, safe_z), 0.0,  0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── box_pocket toolpath ───────────────────────────────────────────────────
// A sharp-corner rect recess.  On a +Z (top-face) box_pocket this is the same
// perimeter-contour path as mill_rect_pocket.  A radial (±X/±Y) box_pocket (a
// side button) is machined in its own LOCAL work-plane frame (radialBoxPocket).
Toolpath boxPocketToolpath(const skill::FeatureSignature& sig)
{
    if (!cutAxisIsZ(sig.params)) return radialBoxPocketToolpath(sig);

    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    // Prefer the world mouth centre (correct for any entry-face position); fall
    // back to the face-local center_*_mm for legacy signatures.
    const double cx    = sig.params.value("center_x_world_mm",
                         sig.params.value("center_x_mm", 0.0));
    const double cy    = sig.params.value("center_y_world_mm",
                         sig.params.value("center_y_mm", 0.0));
    const double L     = sig.params.value("length_mm", 10.0);
    const double W     = sig.params.value("width_mm", 10.0);
    const double depth = sig.params.value("depth_mm", 1.0);
    const double toolD = (sig.tooling.tool_dia_mm > 0.0)
                       ? sig.tooling.tool_dia_mm : std::min(L, W) * 0.4;
    const double toolR = toolD * 0.5;

    const double work_z_top = entryZ(sig.params, "center_z_world_mm");
    const double safe_z = work_z_top + kSafeZAboveStock_mm;
    const double feed   = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                 sig.tooling.feed_per_tooth_mm);
    const double hx = std::max(0.1, L / 2.0 - toolR);
    const double hy = std::max(0.1, W / 2.0 - toolR);

    using M = PathSegment::Move;
    tp.segments.push_back({ M::Rapid, gp_Pnt(cx - hx, cy - hy, safe_z), 0.0, 0, 0, 0 });
    for (double zc : roughingLevels(work_z_top, depth, toolD)) {
        tp.segments.push_back({ M::Linear, gp_Pnt(cx - hx, cy - hy, zc), feed, 0, 0, 0 }); // plunge/step
        tp.segments.push_back({ M::Linear, gp_Pnt(cx + hx, cy - hy, zc), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(cx + hx, cy + hy, zc), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(cx - hx, cy + hy, zc), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(cx - hx, cy - hy, zc), feed, 0, 0, 0 }); // close
    }
    tp.segments.push_back({ M::Rapid, gp_Pnt(cx - hx, cy - hy, safe_z), 0.0, 0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── hole-pattern toolpath (linear_pattern / circular_pattern) ──────────────
// A drilled grid / bolt circle: plunge at each recovered hole centre.  The skills
// emit `hole_centers` (a 3-D entry point per hole) + a cut axis; on the 3-axis-Z
// post only a ±Z pattern is machinable (a side grille is guarded out upstream).
Toolpath holePatternToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double depth = sig.params.value("hole_depth_mm", 0.0);
    const double feed  = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                sig.tooling.feed_per_tooth_mm);

    if (!sig.params.contains("hole_centers") || !sig.params["hole_centers"].is_array()) {
        spdlog::warn("cam: '{}' has no hole_centers — emitting empty toolpath",
                     sig.skill_id);
        return tp;
    }
    using M = PathSegment::Move;
    for (const auto& hc : sig.params["hole_centers"]) {
        if (!hc.is_array() || hc.size() < 2) continue;
        const double x = hc[0].get<double>();
        const double y = hc[1].get<double>();
        const double zTop = (hc.size() >= 3) ? hc[2].get<double>() : 0.0;   // entry Z
        const double safe_z = zTop + kSafeZAboveStock_mm;
        const double cut_z  = zTop - depth;
        // Per-hole: rapid over, plunge to depth, retract.
        tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z), 0.0,  0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(x, y, cut_z),  feed, 0, 0, 0 });
        tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z), 0.0,  0, 0, 0 });
    }
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── counterbore toolpath ──────────────────────────────────────────────────
// A counterbore is a narrow deep PILOT hole with a wider shallow flat-bottomed
// SEAT coaxial on top (the seat clears a socket-head-cap-screw head).  On the
// 3-axis-Z post this is two coaxial plunges at the same (x, y): the pilot to
// pilot_depth_mm, then the seat (a wider tool) to seat_depth_mm.  Params
// (apply + recovered): position_x/y_mm, position_z_mm (entry), axis_dir,
// pilot_dia_mm, pilot_depth_mm, seat_dia_mm, seat_depth_mm.
Toolpath counterboreToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double x         = sig.params.value("position_x_mm", 0.0);
    const double y         = sig.params.value("position_y_mm", 0.0);
    const double zTop      = entryZ(sig.params);   // "position_z_mm", default 0
    const double pilotDep  = sig.params.value("pilot_depth_mm", 0.0);
    const double seatDep   = sig.params.value("seat_depth_mm", 0.0);
    const double feed      = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                    sig.tooling.feed_per_tooth_mm);

    // A radial (side-face) counterbore = the same pilot + seat coaxial plunges,
    // emitted in the feature's own local work-plane frame.
    if (!cutAxisIsZ(sig.params)) {
        emitCoaxialPlungesInLocalFrame(tp, gp_Pnt(x, y, zTop), cutAxisDir(sig.params),
                                       { pilotDep, seatDep }, feed);
        return tp;
    }

    const double safe_z = zTop + kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    // Seat first (the wide, shallow spot-face that a counterbore tool cuts),
    // then the pilot to full depth — machining order for a piloted counterbore
    // is pilot-drill THEN counterbore, but for a re-synthesised toolpath the
    // depth-covering geometry is what matters: two coaxial plunges.
    // [pilot] plunge to pilot_depth.
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),            0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - pilotDep),  feed, 0, 0, 0 });
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),            0.0,  0, 0, 0 });
    // [seat] plunge to the (shallower) seat depth.
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),            0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - seatDep),   feed, 0, 0, 0 });
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),            0.0,  0, 0, 0 });

    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── counterbore-ring-pattern toolpath ─────────────────────────────────────
// The grammar compound: N identical counterbores on a bolt circle (a screw-head
// seat ring).  After an Executor replay the workpiece carries ONLY this compound
// signature — the member counterbore signatures are not kept — so the ring needs
// its own generator: per instance i at angle start + i·360/N on the pitch circle,
// the counterbore plunge pair (seat to seat_depth_mm, pilot to pilot_depth_mm)
// from the stamped entry plane (position_z_mm), all instances in ONE toolpath
// like holePatternToolpath.  Params (apply): count, bolt_circle_dia_mm,
// center_x/y_mm, position_z_mm, start_angle_deg, axis_dir, pilot_dia/depth_mm,
// seat_dia/depth_mm.
Toolpath counterboreRingPatternToolpath(const skill::FeatureSignature& sig)
{
    // The ring skill is authored ±Z-only (its validate rejects a tilted axis);
    // a non-Z compound cannot be machined by the 3-axis-Z post, so fail visibly
    // with the same empty radial-guard toolpath the dispatcher emits for the
    // kZAxisGuarded skills instead of shipping a wrong vertical path.
    if (!cutAxisIsZ(sig.params)) return emptyRadialToolpath(sig);

    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const int    count    = sig.params.value("count", 0);
    const double R        = sig.params.value("bolt_circle_dia_mm", 0.0) / 2.0;
    const double cx       = sig.params.value("center_x_mm", 0.0);
    const double cy       = sig.params.value("center_y_mm", 0.0);
    const double zTop     = entryZ(sig.params);   // "position_z_mm", default 0
    const double startDeg = sig.params.value("start_angle_deg", 0.0);
    const double pilotDep = sig.params.value("pilot_depth_mm", 0.0);
    const double seatDep  = sig.params.value("seat_depth_mm", 0.0);
    const double feed     = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                   sig.tooling.feed_per_tooth_mm);

    const double safe_z = zTop + kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    for (int i = 0; i < count; ++i) {
        const double ang = (startDeg + i * 360.0 / count) * M_PI / 180.0;
        const double x   = cx + R * std::cos(ang);
        const double y   = cy + R * std::sin(ang);
        // Per instance: the counterbore's two coaxial plunges (order is
        // immaterial for a re-synthesised depth-covering path — see the
        // counterbore atom's note).
        // [seat] plunge to the (shallower) seat depth.
        tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),          0.0,  0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - seatDep),  feed, 0, 0, 0 });
        tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),          0.0,  0, 0, 0 });
        // [pilot] plunge to full pilot depth.
        tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),          0.0,  0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - pilotDep), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),          0.0,  0, 0, 0 });
    }
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── countersink toolpath ──────────────────────────────────────────────────
// A countersink is a PILOT hole with a CONICAL seat (a chamfered mouth for a
// flat-head screw).  On the 3-axis-Z post: plunge the pilot to pilot_depth_mm,
// then plunge a countersink (chamfer) tool to cone_depth_mm.  Params:
// position_x/y_mm, position_z_mm, axis_dir, pilot_dia/depth_mm, cone_top_dia_mm,
// cone_angle_deg, cone_depth_mm.
Toolpath countersinkToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double x        = sig.params.value("position_x_mm", 0.0);
    const double y        = sig.params.value("position_y_mm", 0.0);
    const double zTop     = entryZ(sig.params);
    const double pilotDep = sig.params.value("pilot_depth_mm", 0.0);
    const double coneDep  = sig.params.value("cone_depth_mm", 0.0);
    const double feed     = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                   sig.tooling.feed_per_tooth_mm);

    // A radial (side-face) countersink = pilot + chamfer coaxial plunges in the
    // feature's local work-plane frame.
    if (!cutAxisIsZ(sig.params)) {
        emitCoaxialPlungesInLocalFrame(tp, gp_Pnt(x, y, zTop), cutAxisDir(sig.params),
                                       { pilotDep, coneDep }, feed);
        return tp;
    }

    const double safe_z = zTop + kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    // [pilot] plunge to full pilot depth.
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),           0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - pilotDep), feed, 0, 0, 0 });
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),           0.0,  0, 0, 0 });
    // [chamfer] plunge the countersink tool to the cone depth (the tool's cone
    // half-angle equals cone_angle_deg/2; the toolpath just carries the tip Z).
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),           0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - coneDep),  feed, 0, 0, 0 });
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),           0.0,  0, 0, 0 });

    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── ream toolpath ─────────────────────────────────────────────────────────
// A ream is a single finishing pass that enlarges an existing bore to a precise
// diameter.  One plunge along the bore axis to the bore extent.  Unlike the
// hole family, ream carries `axis_location` (a 3-D point ON the axis) rather
// than position_x/y/z, and `extent_mm` (bore length) rather than a depth.
Toolpath reamToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    // Entry point: prefer entry_*_mm (the true bore-surface end the tool enters
    // from), which the ream skill recovers from the actual bore-face span.  Fall
    // back to axis_location only for legacy signatures — note axis_location is the
    // raw OCCT cylinder-axis BASE (a construction artefact offset from the real
    // surface by the cutter overhang, and unbounded for foreign geometry), so it
    // is NOT the entry plane.
    double x = 0.0, y = 0.0, zTop = 0.0;
    if (sig.params.contains("entry_z_mm")) {
        x    = sig.params.value("entry_x_mm", 0.0);
        y    = sig.params.value("entry_y_mm", 0.0);
        zTop = sig.params.value("entry_z_mm", 0.0);
    } else if (sig.params.contains("axis_location") && sig.params["axis_location"].is_array()
               && sig.params["axis_location"].size() == 3) {
        const auto& p = sig.params["axis_location"];
        x = p[0].get<double>(); y = p[1].get<double>(); zTop = p[2].get<double>();
    }
    const double extent = sig.params.value("extent_mm", 0.0);
    const double feed   = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                 sig.tooling.feed_per_tooth_mm);

    // A radial (side-face) ream = a single finishing plunge along the bore axis in
    // the feature's local work-plane frame.
    if (!cutAxisIsZ(sig.params)) {
        emitCoaxialPlungesInLocalFrame(tp, gp_Pnt(x, y, zTop), cutAxisDir(sig.params),
                                       { extent }, feed);
        return tp;
    }

    const double safe_z = zTop + kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    // Single finishing plunge the length of the bore, then retract.
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),          0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - extent),  feed, 0, 0, 0 });
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),          0.0,  0, 0, 0 });

    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── bore_with_shelf toolpath ──────────────────────────────────────────────
// A stepped bore: a wider/narrower UPPER bore to upper_depth, then a coaxial
// LOWER bore to (upper_depth + lower_depth).  lower_depth is measured FROM THE
// SHELF, so the deep pass reaches upper_depth + lower_depth below the entry.
// Two coaxial plunges (rough two-tool boring), same shape as counterbore.
Toolpath boreWithShelfToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double x        = sig.params.value("position_x_mm", 0.0);
    const double y        = sig.params.value("position_y_mm", 0.0);
    const double zTop     = entryZ(sig.params);
    const double upDep    = sig.params.value("upper_depth_mm", 0.0);
    const double loDep    = sig.params.value("lower_depth_mm", 0.0);
    const double feed     = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                   sig.tooling.feed_per_tooth_mm);

    // A radial (side-face) stepped bore = the upper + full-depth coaxial plunges
    // emitted in the feature's own local work-plane frame.
    if (!cutAxisIsZ(sig.params)) {
        emitCoaxialPlungesInLocalFrame(tp, gp_Pnt(x, y, zTop), cutAxisDir(sig.params),
                                       { upDep, upDep + loDep }, feed);
        return tp;
    }

    const double safe_z = zTop + kSafeZAboveStock_mm;
    using M = PathSegment::Move;
    // [upper] plunge to the shelf (upper_depth below entry).
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),               0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - upDep),         feed, 0, 0, 0 });
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),               0.0,  0, 0, 0 });
    // [lower] plunge to the full depth (shelf + lower_depth below entry).
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),               0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - (upDep + loDep)), feed, 0, 0, 0 });
    tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),               0.0,  0, 0, 0 });

    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── multi_step_bore toolpath ──────────────────────────────────────────────
// N coaxial steps of {dia_mm, depth_mm}; steps[0] is the entry (widest) and the
// depths are CUMULATIVE down the bore.  A plunge at the CUMULATIVE depth of each
// step (each pass a different-diameter boring bar).  3 segments per step.
Toolpath multiStepBoreToolpath(const skill::FeatureSignature& sig)
{
    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double x    = sig.params.value("position_x_mm", 0.0);
    const double y    = sig.params.value("position_y_mm", 0.0);
    const double zTop = entryZ(sig.params);
    const double feed = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                               sig.tooling.feed_per_tooth_mm);

    const double safe_z = zTop + kSafeZAboveStock_mm;
    if (!sig.params.contains("steps") || !sig.params["steps"].is_array()) {
        spdlog::warn("cam: '{}' has no steps — emitting empty toolpath", sig.skill_id);
        return tp;
    }

    // A radial (side-face) multi-step bore = one coaxial plunge per step at the
    // CUMULATIVE depth, emitted in the feature's own local work-plane frame.
    if (!cutAxisIsZ(sig.params)) {
        std::vector<double> depths;
        double cum = 0.0;
        for (const auto& s : sig.params["steps"]) { cum += s.value("depth_mm", 0.0); depths.push_back(cum); }
        emitCoaxialPlungesInLocalFrame(tp, gp_Pnt(x, y, zTop), cutAxisDir(sig.params), depths, feed);
        return tp;
    }

    using M = PathSegment::Move;
    double cumulative = 0.0;
    for (const auto& s : sig.params["steps"]) {
        cumulative += s.value("depth_mm", 0.0);   // depths are cumulative down the bore
        tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),            0.0,  0, 0, 0 });
        tp.segments.push_back({ M::Linear, gp_Pnt(x, y, zTop - cumulative), feed, 0, 0, 0 });
        tp.segments.push_back({ M::Rapid,  gp_Pnt(x, y, safe_z),            0.0,  0, 0, 0 });
    }
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── box_boss toolpath ─────────────────────────────────────────────────────
// A raised rectangular island: the INVERSE of box_pocket.  Instead of cutting a
// recess, we clear the surrounding field down to the boss base plane, leaving the
// boss standing.  Slice-1 minimum-viable = a single finishing profile contour
// that traces the boss footprint offset OUTWARD by the tool radius, at the base
// plane depth (the tool skirts the wall, removing the adjacent field).  The boss
// stands height_mm above center_z_world_mm; stock top = base + height.
Toolpath boxBossToolpath(const skill::FeatureSignature& sig)
{
    // A radial (side-face) box boss = the same outward field-clearing rectangle in
    // the feature's own local work-plane frame.
    if (!cutAxisIsZ(sig.params)) return radialBoxBossToolpath(sig);

    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double cx     = sig.params.value("center_x_world_mm", 0.0);
    const double cy     = sig.params.value("center_y_world_mm", 0.0);
    const double baseZ  = sig.params.value("center_z_world_mm", 0.0);
    const double L      = sig.params.value("length_mm", 10.0);
    const double W      = sig.params.value("width_mm", 10.0);
    const double height = sig.params.value("height_mm", 1.0);
    const double toolD  = (sig.tooling.tool_dia_mm > 0.0)
                        ? sig.tooling.tool_dia_mm : std::min(L, W) * 0.4;
    const double toolR  = toolD * 0.5;
    const double feed   = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                 sig.tooling.feed_per_tooth_mm);

    // Stock top is at the boss top; retract above it.
    const double safe_z = baseZ + height + kSafeZAboveStock_mm;
    // Contour OUTSIDE the boss wall by the tool radius (skirts the wall).
    const double hx = L / 2.0 + toolR;
    const double hy = W / 2.0 + toolR;
    const double zc = baseZ;   // clear the field down to the base plane

    using M = PathSegment::Move;
    tp.segments.push_back({ M::Rapid,  gp_Pnt(cx - hx, cy - hy, safe_z), 0.0,  0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(cx - hx, cy - hy, zc),     feed, 0, 0, 0 }); // plunge
    tp.segments.push_back({ M::Linear, gp_Pnt(cx + hx, cy - hy, zc),     feed, 0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(cx + hx, cy + hy, zc),     feed, 0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(cx - hx, cy + hy, zc),     feed, 0, 0, 0 });
    tp.segments.push_back({ M::Linear, gp_Pnt(cx - hx, cy - hy, zc),     feed, 0, 0, 0 }); // close
    tp.segments.push_back({ M::Rapid,  gp_Pnt(cx - hx, cy - hy, safe_z), 0.0,  0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── dome_boss toolpath ────────────────────────────────────────────────────
// A raised spherical-cap island, finished with a ball-nose 3-D pass: a stack of
// constant-Z rings (Z-level / waterline finishing) from the apex down to the
// base, each ring at the dome's SURFACE radius for its height.  Below every ring
// a final skirt contour offset OUTWARD by the tool radius clears the field and
// defines the footprint.
//
// Geometry: the cap is a sphere of radius Rs = (r² + h²) / 2h whose apex is at
// base_z + h and whose base circle (radius r) is at base_z.  At a cut height zc
// (base_z ≤ zc ≤ base_z + h), let d = (base_z + h) − zc be the drop below the
// apex; the SURFACE radius is  rho(zc) = sqrt(Rs² − (Rs − d)²) = sqrt(2·Rs·d − d²).
// The BALL-CENTRE path (what the toolpath drives) is offset toolR along the
// outward surface normal = radially from the sphere centre, so it is a concentric
// sphere of radius Rs + toolR: each surface ring is scaled by (Rs+toolR)/Rs about
// the sphere centre.  (Driving the tool TIP along the surface would gouge by toolR.)
Toolpath domeBossToolpath(const skill::FeatureSignature& sig)
{
    // A radial (side-face) dome boss = the same ball-nose ring stack in the
    // feature's own local work-plane frame.
    if (!cutAxisIsZ(sig.params)) return radialDomeBossToolpath(sig);

    Toolpath tp;
    tp.feature_skill_id = sig.skill_id;
    tp.tooling          = sig.tooling;
    tp.tool_id          = makeToolId(sig.tooling);
    tp.tool_dia_mm      = sig.tooling.tool_dia_mm;
    tp.tool_length_mm   = sig.tooling.tool_length_mm;
    tp.spindle_rpm      = sfmToRpm(sig.tooling.cutting_speed_sfm, sig.tooling.tool_dia_mm);

    const double cx     = sig.params.value("center_x_world_mm", 0.0);
    const double cy     = sig.params.value("center_y_world_mm", 0.0);
    const double baseZ  = sig.params.value("center_z_world_mm", 0.0);
    const double baseR  = sig.params.value("base_radius_mm", 5.0);
    const double height = sig.params.value("height_mm", 1.0);
    const double toolD  = (sig.tooling.tool_dia_mm > 0.0)
                        ? sig.tooling.tool_dia_mm : baseR * 0.4;
    const double toolR  = toolD * 0.5;
    const double feed   = computeFeed_mm_per_min(tp.spindle_rpm, sig.tooling.flute_count,
                                                 sig.tooling.feed_per_tooth_mm);

    const double safe_z = baseZ + height + kSafeZAboveStock_mm;
    const double apexZ  = baseZ + height;
    const double Rs     = (height > 1e-6) ? (baseR * baseR + height * height) / (2.0 * height)
                                          : baseR;   // degenerate flat → treat as a disc

    using M = PathSegment::Move;

    // Emit one full circle (four ArcCCW quarter-turns) of radius `r` at height z,
    // starting/ending at (cx + r, cy).  Assumes the tool is already positioned at
    // the ring start; returns the final position.
    auto emitRing = [&](double r, double z) {
        const double quarter[4][2] = { {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}, {1.0, 0.0} };
        gp_Pnt cur(cx + r, cy, z);
        for (int q = 0; q < 4; ++q) {
            const double nx = cx + r * quarter[q][0];
            const double ny = cy + r * quarter[q][1];
            const double ic = cx - cur.X();   // centre - start
            const double jc = cy - cur.Y();
            tp.segments.push_back({ M::ArcCCW, gp_Pnt(nx, ny, z), feed, ic, jc, 0.0 });
            cur = gp_Pnt(nx, ny, z);
        }
    };

    // Z-level rings from just below the apex down to the base.  Stepover = a
    // quarter of the tool diameter (typical ball-nose finishing scallop).
    const double stepover = std::max(0.2, toolD * 0.25);
    const int    nLevels  = std::max(1, static_cast<int>(std::ceil(height / stepover)));

    // BALL-NOSE offset: the cap is part of a sphere of radius Rs centred at
    // sphereCenterZ on the axis.  The ball CENTRE (the point the toolpath drives)
    // touches the surface at toolR along the outward surface normal, which for a
    // sphere is radial — so the ball-centre path is a CONCENTRIC sphere of radius
    // Rs + toolR.  Each surface ring (rho, zc) maps to the ball-centre ring by a
    // uniform scale (Rs + toolR)/Rs about the sphere centre.  (The old tip-contact
    // path drove the tip along the surface, leaving a toolR gouge/undercut.)
    const double sphereCenterZ = baseZ + height - Rs;
    const double scale         = (Rs > 1e-6) ? (Rs + toolR) / Rs : 1.0;

    // Rapid to the first ring start above the apex, then descend ring by ring.
    // Level k (1..nLevels) sits at contact zc = apexZ - k*(height/nLevels); the
    // surface radius grows from ~0 at the apex to baseR at the base.
    bool positioned = false;
    for (int k = 1; k <= nLevels; ++k) {
        const double zc  = apexZ - (height * static_cast<double>(k) / static_cast<double>(nLevels));
        const double d   = apexZ - zc;                       // drop below apex
        const double rho = std::sqrt(std::max(0.0, 2.0 * Rs * d - d * d));  // surface radius
        // Ball-centre ring: scale the surface ring outward from the sphere centre.
        const double r  = std::max(0.1, rho * scale);
        // For a > hemisphere (tall) dome the sub-equator rings scale to BELOW the
        // base plane; clamp so the ball centre never drives past the base (the
        // field-clearing skirt owns the base plane).  For the shallow domes we
        // build (a watch crown: baseR >> height, sub-hemisphere) zb is already
        // ≥ baseZ so the clamp is a no-op.
        const double zb = std::max(baseZ, sphereCenterZ + (zc - sphereCenterZ) * scale);
        if (!positioned) {
            tp.segments.push_back({ M::Rapid,  gp_Pnt(cx + r, cy, safe_z), 0.0,  0, 0, 0 });
            tp.segments.push_back({ M::Linear, gp_Pnt(cx + r, cy, zb),     feed, 0, 0, 0 });
            positioned = true;
        } else {
            // Move (at feed) out to this ring's radius on the +X spoke, at its Z.
            tp.segments.push_back({ M::Linear, gp_Pnt(cx + r, cy, zb), feed, 0, 0, 0 });
        }
        emitRing(r, zb);
    }

    // Final skirt contour at the base plane, offset OUTWARD by the tool radius —
    // clears the surrounding field and defines the footprint.  This is a field-
    // clearing pass (the tool side edge cuts down to the base), so it stays at the
    // surface base plane, not the lifted ball-centre Z.
    const double skirtR = baseR + toolR;
    tp.segments.push_back({ M::Linear, gp_Pnt(cx + skirtR, cy, baseZ), feed, 0, 0, 0 });
    emitRing(skirtR, baseZ);

    tp.segments.push_back({ M::Rapid, gp_Pnt(cx + skirtR, cy, safe_z), 0.0, 0, 0, 0 }); // retract
    tp.est_cycle_time_s = estimateCycleTime_s(tp.segments, feed);
    return tp;
}

// ── Dispatcher ───────────────────────────────────────────────────────────

std::vector<Toolpath> generateAllToolpaths(
    const std::vector<skill::FeatureSignature>& sigs)
{
    std::vector<Toolpath> out;
    out.reserve(sigs.size());
    for (const auto& sig : sigs) {
        // Guard: a recovered RADIAL (non-Z cut axis) feature cannot be machined by
        // a pure 3-axis-Z post in the GLOBAL frame.  Two responses by feature type:
        //   - a plain axial hole/bore (a side bore, side drill) IS a simple plunge
        //     — we emit it in the feature's LOCAL work-plane frame + record the
        //     frame (radialDrillToolpath) so a re-setup / post can reach it;
        //   - a radial pocket / pattern / boss needs an in-plane contour on a side
        //     face — still deferred to an empty toolpath + warning.
        static const std::set<std::string> kRadialDrillable = {
            "drill_hole", "drill_through_hole", "spot_drill", "spot_face",
            "bore_cylindrical", "micro_drill", "gun_drill",
        };
        // Radial patterns (side grille / side bolt circle) → per-hole plunges in
        // the pattern's local work-plane frame.  (box_pocket routes its own radial
        // case inside boxPocketToolpath → radialBoxPocketToolpath.)
        static const std::set<std::string> kRadialContour = {
            "linear_pattern", "circular_pattern",
        };
        // Nearly the whole hole/bore/pocket/slot/boss family now handles its OWN
        // radial case internally (routing to a local-frame generator), like
        // box_pocket — so those are NOT in this deferred set.  What remains is only
        // mill_rect_pocket, which CANNOT be authored radial (its apply() rejects a
        // non-Z axis and recognize() hard-codes -Z), and box_pocket already covers
        // side rectangular recesses — so a radial mill_rect_pocket is unreachable
        // and an empty toolpath is the safe fallback.
        static const std::set<std::string> kZAxisGuarded = {
            "mill_rect_pocket",
        };
        if (!cutAxisIsZ(sig.params)) {
            if (kRadialDrillable.count(sig.skill_id)) {
                out.push_back(radialDrillToolpath(sig));
                continue;
            }
            if (kRadialContour.count(sig.skill_id)) {
                out.push_back(radialHolePatternToolpath(sig));
                continue;
            }
            if (kZAxisGuarded.count(sig.skill_id)) {
                out.push_back(emptyRadialToolpath(sig));
                continue;
            }
        }
        if (sig.skill_id == "drill_hole") {
            out.push_back(drillHoleToolpath(sig));
        } else if (sig.skill_id == "mill_circular_pocket") {
            out.push_back(millCircularPocketToolpath(sig));
        } else if (sig.skill_id == "mill_rect_pocket") {
            out.push_back(millRectPocketToolpath(sig));
        } else if (sig.skill_id == "mill_slot") {
            out.push_back(millSlotToolpath(sig));
        } else if (sig.skill_id == "box_pocket") {
            out.push_back(boxPocketToolpath(sig));
        } else if (sig.skill_id == "box_boss") {
            out.push_back(boxBossToolpath(sig));
        } else if (sig.skill_id == "dome_boss") {
            out.push_back(domeBossToolpath(sig));
        } else if (sig.skill_id == "linear_pattern" ||
                   sig.skill_id == "circular_pattern") {
            out.push_back(holePatternToolpath(sig));
        } else if (sig.skill_id == "counterbore_ring_pattern") {
            // The grammar compound survives replay ALONE (member counterbore
            // sigs are dropped); the generator regenerates the per-instance
            // plunges from the ring params.  ±Z-only — it guards its own
            // radial case to the empty toolpath internally.
            out.push_back(counterboreRingPatternToolpath(sig));
        } else if (sig.skill_id == "bore_cylindrical"   ||
                   sig.skill_id == "drill_through_hole" ||
                   sig.skill_id == "spot_drill"         ||
                   sig.skill_id == "spot_face"          ||
                   sig.skill_id == "micro_drill"        ||
                   sig.skill_id == "gun_drill") {
            // The plain-hole bore/drill family is a plunge at a position to a
            // depth — the drill_hole toolpath shape covers them directly (they
            // carry position_x/y + depth + diameter).  micro_drill (sub-mm) and
            // gun_drill (deep-hole) differ only in tool/strategy, not path shape.
            out.push_back(drillHoleToolpath(sig));
        } else if (sig.skill_id == "counterbore") {
            out.push_back(counterboreToolpath(sig));
        } else if (sig.skill_id == "countersink") {
            out.push_back(countersinkToolpath(sig));
        } else if (sig.skill_id == "ream") {
            out.push_back(reamToolpath(sig));
        } else if (sig.skill_id == "bore_with_shelf") {
            out.push_back(boreWithShelfToolpath(sig));
        } else if (sig.skill_id == "multi_step_bore") {
            out.push_back(multiStepBoreToolpath(sig));
        } else {
            spdlog::warn("cam::generateAllToolpaths: no generator for skill_id '{}'; "
                         "emitting empty toolpath", sig.skill_id);
            Toolpath empty;
            empty.feature_skill_id = sig.skill_id;
            empty.tool_id          = makeToolId(sig.tooling);
            empty.tool_dia_mm      = sig.tooling.tool_dia_mm;
            empty.tool_length_mm   = sig.tooling.tool_length_mm;
            out.push_back(std::move(empty));
        }
    }
    return out;
}

// ── G-code export ────────────────────────────────────────────────────────

// Map a LOCAL (u, v, depth) point to WORLD via the toolpath's work-plane frame:
// world = work_origin + u·work_u + v·work_v + depth·work_depth.  (A rigid-body
// transform R·p + t, R = [u|v|depth] columns, t = work_origin.)  Manual gp_Vec
// composition — no gp_Trsf, and no gp_Dir::Dot(gp_Vec) normalisation footgun.
gp_Pnt localToWorldPoint(const gp_Pnt& p, const Toolpath& tp)
{
    const gp_Vec off = gp_Vec(tp.work_u_axis)     * p.X()
                     + gp_Vec(tp.work_v_axis)     * p.Y()
                     + gp_Vec(tp.work_depth_axis) * p.Z();
    return gp_Pnt(tp.work_origin.X() + off.X(),
                  tp.work_origin.Y() + off.Y(),
                  tp.work_origin.Z() + off.Z());
}

// ZYX Euler angles (degrees) of the work-plane frame R = [u | v | depth] (columns),
// i.e. the rotation that maps the machine axes onto the tilted work plane.  Fanuc
// G68.2 P0 and Heidenhain CYCL DEF 19 both take this triple; we emit it as a
// COMMENTED alternative to the world-transformed lines so a 5-axis / tilted-head
// controller can re-post the feature as native tilted-plane moves (with true G2/G3
// arcs) instead of the linearised world chords.  Convention: R = Rz(K)·Ry(J)·Rx(I),
// so K spins about Z, then J about the new Y, then I about the new X.  The gimbal-
// lock branch (|R20|→1) folds I into K, the standard degenerate handling.
struct WorkPlaneEuler { double i_deg, j_deg, k_deg; };
WorkPlaneEuler workPlaneEulerZYX(const Toolpath& tp)
{
    // Column vectors of R.  R[row][col]: r_c0 = u, r_c1 = v, r_c2 = depth.
    const gp_Dir& u = tp.work_u_axis;
    const gp_Dir& v = tp.work_v_axis;
    const gp_Dir& w = tp.work_depth_axis;
    const double r00 = u.X(), r10 = u.Y(), r20 = u.Z();
    const double r21 = v.Z();
    const double r22 = w.Z();
    const double kRad2Deg = 180.0 / M_PI;
    const double cy = std::hypot(r00, r10);   // cos(J) = sqrt(r00²+r10²)
    double iRad, jRad, kRad;
    if (cy > 1e-9) {
        jRad = std::atan2(-r20, cy);
        kRad = std::atan2(r10, r00);
        iRad = std::atan2(r21, r22);
    } else {
        // Gimbal lock: J = ±90°, X and Z axes align — fold I into K, set I = 0.
        jRad = std::atan2(-r20, cy);
        kRad = std::atan2(-v.X(), v.Y());
        iRad = 0.0;
    }
    return { iRad * kRad2Deg, jRad * kRad2Deg, kRad * kRad2Deg };
}

// One segment's world end point + a coarse linearisation of a radial ARC into
// straight world segments.  A radial (tilted-plane) arc is NOT expressible as a
// single G2/G3 with I/J on a 3-axis controller (the arc plane is the work plane,
// not G17/G18/G19), so we linearise it into short chords in WORLD coordinates —
// always correct and dialect-agnostic.  Returns the list of world points to emit
// as G1 moves (for a radial arc) or a single world point (Rapid/Linear or a +Z
// path where the arc is emitted natively as G2/G3 elsewhere).
constexpr int kArcLinearizeSteps = 8;   // chords per quarter-turn arc segment

// One toolpath's setup + moves (tool comment, G21/G90/G17, M3 S, optional radial
// SETUP banner, the G0/G1/G2/G3 blocks, M5).  NO program-end (M30) and no
// %/program wrapper — so this composes into a multi-toolpath program.  A leading
// M5 is emitted only when the spindle is on; the caller decides program framing.
//
// For a RADIAL toolpath every coordinate is transformed LOCAL → WORLD before it
// is emitted, so the G-code is runnable world XYZ (the SETUP comment stays as
// metadata documenting the work plane).  Radial arcs are linearised into world
// line segments (a tilted-plane arc has no single G17/18/19 plane).  The
// Toolpath.segments themselves are left untouched (still local) — only the
// emitted text is world.
void emitToolpathBody(const Toolpath& path, std::ostringstream& s)
{
    // Header.
    s << "(feature: " << path.feature_skill_id << ")\n";
    s << "(TOOL: " << path.tool_id
      << "  dia=" << path.tool_dia_mm << "mm"
      << "  len=" << path.tool_length_mm << "mm)\n";
    s << "(SPINDLE: " << static_cast<long long>(std::llround(path.spindle_rpm)) << " rpm)\n";
    s << "G21\n";  // metric
    s << "G90\n";  // absolute
    s << "G17\n";  // XY plane — REQUIRED so G2/G3 arc I/J are unambiguous
    // Spindle on (clockwise) at the computed RPM.  Without M3 S the tool does not
    // rotate and the program will not cut; emit it whenever we have a spindle speed.
    if (path.spindle_rpm > 0.0)
        s << "M3 S" << static_cast<long long>(std::llround(path.spindle_rpm)) << "\n";

    // Radial / tilted feature: the moves below are the feature's local (u,v,depth)
    // path TRANSFORMED to WORLD XYZ (so the program is runnable on a 3-axis post
    // with the part in place).  The work-plane frame is kept as a comment so a
    // re-setup / fixture can be verified and radial arcs (linearised into world
    // chords here) can be regenerated as native arcs if a controller supports the
    // tilted plane (G68.2 / CYCL DEF 19).
    if (path.is_radial()) {
        s << "(SETUP: RADIAL feature — coordinates below are WORLD, transformed "
             "from the local work plane)\n";
        s << "(WORK_ORIGIN: X" << path.work_origin.X()
          << " Y" << path.work_origin.Y() << " Z" << path.work_origin.Z() << ")\n";
        s << "(WORK_U: " << path.work_u_axis.X() << " " << path.work_u_axis.Y()
          << " " << path.work_u_axis.Z() << ")\n";
        s << "(WORK_V: " << path.work_v_axis.X() << " " << path.work_v_axis.Y()
          << " " << path.work_v_axis.Z() << ")\n";
        s << "(WORK_DEPTH: " << path.work_depth_axis.X() << " " << path.work_depth_axis.Y()
          << " " << path.work_depth_axis.Z() << ")\n";
        // Native tilted-work-plane alternative, emitted as COMMENTS so it changes
        // nothing for a 3-axis post (the runnable moves stay world XYZ below).  A
        // 5-axis / tilted-head controller can enable these to re-cut the feature on
        // the work plane with true G2/G3 arcs instead of the linearised chords.
        const WorkPlaneEuler e = workPlaneEulerZYX(path);
        s << "(ALT: tilted work plane — enable ONE dialect on a capable control)\n";
        s << "(ALT Fanuc:  G68.2 X" << path.work_origin.X() << " Y" << path.work_origin.Y()
          << " Z" << path.work_origin.Z()
          << " I" << e.i_deg << " J" << e.j_deg << " K" << e.k_deg << " ; then G53.1)\n";
        s << "(ALT Heidenhain: CYCL DEF 19.0 WORKING PLANE / 19.1 A" << e.i_deg
          << " B" << e.j_deg << " C" << e.k_deg << ")\n";
    }

    const bool radial = path.is_radial();
    gp_Pnt prevLocal(0.0, 0.0, 0.0);   // segment start = previous segment's end
    bool havePrev = false;
    for (const auto& seg : path.segments) {
        const bool isArc = (seg.move == PathSegment::Move::ArcCW ||
                            seg.move == PathSegment::Move::ArcCCW);

        // Emit one G0/G1 move to a WORLD point (transforming when radial).
        auto emitMove = [&](const char* code, const gp_Pnt& localPt, double feed) {
            const gp_Pnt p = radial ? localToWorldPoint(localPt, path) : localPt;
            s << code << " X" << p.X() << " Y" << p.Y() << " Z" << p.Z();
            if (feed > 0.0) s << " F" << feed;
            s << "\n";
        };

        if (radial && isArc && havePrev) {
            // Linearise the arc in the LOCAL plane, transforming each chord point
            // to world.  Arc centre (local) = start + (arc_i, arc_j, arc_k).
            const gp_Pnt c(prevLocal.X() + seg.arc_i, prevLocal.Y() + seg.arc_j,
                           prevLocal.Z() + seg.arc_k);
            const double a0 = std::atan2(prevLocal.Y() - c.Y(), prevLocal.X() - c.X());
            double a1 = std::atan2(seg.end_point.Y() - c.Y(), seg.end_point.X() - c.X());
            const bool ccw = (seg.move == PathSegment::Move::ArcCCW);
            if (ccw && a1 <= a0) a1 += 2.0 * M_PI;
            if (!ccw && a1 >= a0) a1 -= 2.0 * M_PI;
            const double r  = std::hypot(prevLocal.X() - c.X(), prevLocal.Y() - c.Y());
            const double zA = prevLocal.Z(), zB = seg.end_point.Z();
            for (int k = 1; k <= kArcLinearizeSteps; ++k) {
                const double t  = static_cast<double>(k) / kArcLinearizeSteps;
                const double an = a0 + (a1 - a0) * t;
                const gp_Pnt chord(c.X() + r * std::cos(an), c.Y() + r * std::sin(an),
                                   zA + (zB - zA) * t);
                emitMove("G1", chord, seg.feed_mm_per_min);
            }
        } else {
            const char* code = seg.move == PathSegment::Move::Rapid  ? "G0"
                             : seg.move == PathSegment::Move::Linear ? "G1"
                             : seg.move == PathSegment::Move::ArcCW  ? "G2" : "G3";
            if (isArc && !radial) {
                // +Z arc: native G2/G3 with I/J/K (relative offsets, world already).
                s << code << " X" << seg.end_point.X() << " Y" << seg.end_point.Y()
                  << " Z" << seg.end_point.Z()
                  << " I" << seg.arc_i << " J" << seg.arc_j << " K" << seg.arc_k;
                if (seg.feed_mm_per_min > 0.0) s << " F" << seg.feed_mm_per_min;
                s << "\n";
            } else {
                emitMove(code, seg.end_point, seg.move == PathSegment::Move::Rapid
                                              ? 0.0 : seg.feed_mm_per_min);
            }
        }
        prevLocal = seg.end_point;
        havePrev  = true;
    }

    if (path.spindle_rpm > 0.0) s << "M5\n";   // spindle off after this feature
}

std::string toGCode(const Toolpath& path)
{
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s.precision(3);

    s << "(KooCADCAM slice-1 G-code)\n";
    emitToolpathBody(path, s);
    s << "M30\n";  // program end (used in place of M02 — both valid in ISO 6983)
    return s.str();
}

std::string toGCodeProgram(const std::vector<Toolpath>& paths, bool ok, int collisionCount)
{
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s.precision(3);

    s << "%\n";
    s << "(KooCADCAM G-code program — " << paths.size() << " toolpath(s)";
    if (!ok) s << "; WARNING " << collisionCount << " collision(s)";
    s << ")\n";
    // Each toolpath's body (header + moves + its own M5); a (TOOLCHANGE ...) comment
    // marks a change of tool between features so a post can insert T/M6.  A single
    // program-end (M30) closes the WHOLE program — NOT one per toolpath (that would
    // stop the machine at the first feature).
    // A tool change between features: look the requirement up in the magazine and
    // emit an EXECUTABLE `T<n> M6` (the descriptive label stays as a comment).  When
    // the magazine has no matching pocket we fall back to the comment-only form — a
    // post must never machine a feature with a silently-wrong tool.  We key the
    // "did the tool change" test on the resolved magazine pocket (T-number) when we
    // have one, so two features that map to the SAME pocket don't re-emit a change.
    std::string prevKey;
    for (const auto& tp : paths) {
        if (!tp.tool_id.empty()) {
            const std::optional<ToolEntry> pick = selectTool(tp.tooling);
            const std::string key = pick ? ("T" + std::to_string(pick->t_number))
                                         : tp.tool_id;
            if (key != prevKey) {
                if (pick)
                    s << "T" << pick->t_number << " M6 (TOOLCHANGE: " << pick->label
                      << " — " << tp.tool_id << ")\n";
                else
                    s << "(TOOLCHANGE: " << tp.tool_id
                      << " — no magazine pocket; set tool manually)\n";
                prevKey = key;
            }
        }
        emitToolpathBody(tp, s);
    }
    s << "M30\n%\n";
    return s.str();
}

}  // namespace koocadcam::cam
