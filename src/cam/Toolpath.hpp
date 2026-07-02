#pragma once
// @lat: [[engine/cam-3axis-verify#Pipeline]]
//
// Toolpath — 3-axis G-code-style toolpath generator (slice-1).
//
// This is the *minimum-viable* CAM that consumes a FeatureSignature
// (produced by a skill::apply()) and emits a sequence of PathSegments,
// plus a simple plain-text G-code dump.
//
// Scope (slice-1, in line with lat.md/engine/cam-3axis-verify):
//   - 3-axis linear interpolation only (G0 / G1).
//   - Arc moves (G2 / G3) generated for circular pocket spirals.
//   - No tool-change codes, no canned cycles, no dwell/coolant codes.
//   - Estimated cycle time is a naive Σ(length) / feed; ±30% accuracy.
//
// Dispatch (generateAllToolpaths):
//   skill_id == "drill_hole"            → drillHoleToolpath
//   skill_id == "mill_circular_pocket"  → millCircularPocketToolpath
//   anything else                       → empty toolpath + spdlog warning
//
// Future (NOT in slice-1):
//   - Real tool-library lookup (currently we just stamp `tool_id` from
//     ToolingMeta).
//   - Dwell / coolant control.
//   - Proper kinematic feed scheduling (corner deceleration, look-ahead).
//   - Post-processor dialect adapters (Fanuc / Heidenhain / Siemens).

#include "skills/Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::cam {

// One segment of a 3-axis toolpath.  Position + feed.
struct PathSegment
{
    enum class Move
    {
        Rapid,    // G0 — non-cutting, max feed
        Linear,   // G1 — straight cut at programmed feed
        ArcCW,    // G2 — clockwise arc, center offset in (I, J, K)
        ArcCCW    // G3 — counter-clockwise arc, center offset in (I, J, K)
    };

    Move    move = Move::Rapid;
    gp_Pnt  end_point;                 // end of this segment (start = previous segment's end)
    double  feed_mm_per_min = 0.0;     // 0 = rapid

    // For arcs: relative center offset from the segment START to the arc
    // center.  Z axis (K) is included for completeness; in pure 3-axis XY
    // helices it stays 0 and the helix climb is encoded by the difference
    // between start.Z and end_point.Z.
    double  arc_i = 0.0;
    double  arc_j = 0.0;
    double  arc_k = 0.0;
};

// A full toolpath for one feature operation.
struct Toolpath
{
    std::string              feature_skill_id;   // skill_id of the source signature
    std::string              tool_id;            // "drill_3mm_carbide_2flute" etc.
    double                   tool_dia_mm     = 0.0;
    double                   tool_length_mm  = 0.0;
    double                   spindle_rpm     = 0.0;
    double                   coolant_pressure = 0.0;  // 0 = off
    std::vector<PathSegment> segments;
    double                   est_cycle_time_s = 0.0;  // wall-clock estimate
};

// ── Single-feature generators ────────────────────────────────────────────

// Build a toolpath for a single drill_hole feature.
//
// Structure (4 segments):
//   [0]  Rapid    : (x, y, safe_z)
//   [1]  Rapid    : (x, y, work_z + 1mm)          ← approach to clearance
//   [2]  Linear   : (x, y, work_z - depth_mm)     ← plunge at programmed feed
//   [3]  Rapid    : (x, y, safe_z)                ← retract
Toolpath drillHoleToolpath(const skill::FeatureSignature& sig);

// Build a toolpath for a single mill_circular_pocket feature.
//
// Structure:
//   [0..1] approach (rapid to clearance, then to entry)
//   [2..N] helical entry (coarse-discretised ArcCCW + dZ linear) to depth
//   [N+1..M] spiral-out filling the pocket to (diameter - tool_dia) / 2
//   [M+1]  retract (rapid up to safe_z)
//
// Result: ~10-30 segments depending on diameter / step parameters.
Toolpath millCircularPocketToolpath(const skill::FeatureSignature& sig);

// mill_rect_pocket → a finishing perimeter contour offset inward by the tool
// radius; mill_slot → the centreline traverse (start → end at depth).  Both are
// features the recognizer already produces but slice-1 CAM could not machine.
Toolpath millRectPocketToolpath(const skill::FeatureSignature& sig);
Toolpath millSlotToolpath(const skill::FeatureSignature& sig);

// box_pocket → a sharp-corner rect recess.  On the 3-axis-Z post this is
// machinable ONLY when its face_normal is +Z (a top-face pocket: phone camera
// island, watch top pocket) — the same perimeter contour as mill_rect_pocket.
// A RADIAL box_pocket (side button, face_normal ±X/±Y) needs a re-setup / 4th
// axis the post does not have, so it emits an empty toolpath + a warning.
Toolpath boxPocketToolpath(const skill::FeatureSignature& sig);

// linear_pattern / circular_pattern → a drilled grid or bolt circle: a plunge at
// each recovered hole centre (the skills emit `hole_centers`).  Only a ±Z pattern
// is machinable on the 3-axis-Z post (a radial side grille is guarded out).
Toolpath holePatternToolpath(const skill::FeatureSignature& sig);

// counterbore → two coaxial plunges (pilot to pilot_depth_mm, then a wider seat
// to seat_depth_mm).  countersink → pilot plunge + a chamfer-tool plunge to
// cone_depth_mm.  ream → a single finishing plunge along the bore axis to
// extent_mm (anchored on `axis_location`, not position_x/y/z).  All three are
// ±Z-only on the 3-axis post (radial variants are guarded out to empty).
Toolpath counterboreToolpath(const skill::FeatureSignature& sig);
Toolpath countersinkToolpath(const skill::FeatureSignature& sig);
Toolpath reamToolpath(const skill::FeatureSignature& sig);

// bore_with_shelf → two coaxial plunges (upper to upper_depth_mm, then lower to
// upper_depth + lower_depth, since lower_depth is measured from the shelf).
// multi_step_bore → one plunge per step at the CUMULATIVE depth (steps[] carry
// {dia_mm, depth_mm}, depths accumulate down the bore).  Both ±Z-guarded.
Toolpath boreWithShelfToolpath(const skill::FeatureSignature& sig);
Toolpath multiStepBoreToolpath(const skill::FeatureSignature& sig);

// box_boss / dome_boss → a raised island (the INVERSE of a pocket): a finishing
// profile contour offset OUTWARD by the tool radius, at the base plane, that
// clears the surrounding field and leaves the boss standing.  box_boss traces a
// rectangle (length+2·toolR × width+2·toolR); dome_boss a circle (base_radius +
// toolR).  Only a +Z-normal (top-face) boss is machinable; radial is guarded out.
Toolpath boxBossToolpath(const skill::FeatureSignature& sig);
Toolpath domeBossToolpath(const skill::FeatureSignature& sig);

// ── Multi-feature dispatcher ─────────────────────────────────────────────

// Generate toolpaths for ALL features in a vector of signatures (e.g.,
// from `Workpiece::features()` or a `ProcessPlan`).
//
// Unknown skill_ids produce an empty toolpath (segments empty,
// feature_skill_id set) and a spdlog warning.  This lets callers preserve
// 1:1 alignment with the input list while still surfacing missing CAM
// support.
std::vector<Toolpath> generateAllToolpaths(
    const std::vector<skill::FeatureSignature>& sigs);

// ── G-code export ────────────────────────────────────────────────────────

// Export a single toolpath to plain-text G-code.
//
// Output is one move per line, comments prefixed with '(' as per ISO 6983.
// The header includes a `(TOOL: ...)` / `(SPINDLE: ...)` line for human
// readability; no M-codes or T-codes are emitted.  Coordinates are in
// millimetres, formatted with %g precision.
//
// Move codes used:  G0 (Rapid), G1 (Linear), G2 (ArcCW), G3 (ArcCCW).
// G2/G3 emit I/J/K arc-center offsets relative to the segment start.
std::string toGCode(const Toolpath& path);

}  // namespace koocadcam::cam
