#pragma once
// @lat: [[engine/skills#cmm_scan]]
//
// cmm_scan — INSPECTION skill that records a coordinate-measuring-machine
// (CMM) waypoint program: a list of N touch waypoints, each with a
// nominal world-XYZ location and a per-point measured value.  The
// FeatureSignature records the full waypoint table plus the
// measured-vs-nominal delta so downstream consumers can render a
// production CMM macro and verify the part is within target_tolerance.
//
// Geometrically a NO-OP — like probe_point and scan_pattern, the part
// shape is unchanged.  Differentiation:
//   - probe_point     : single touch, geometry-bounded location check
//   - scan_pattern    : N waypoints generated from a face + pattern rule
//   - cmm_scan        : N caller-supplied waypoints with nominal/measured
//                       deltas → maps directly to a CMM measurement
//                       report (PC-DMIS, Calypso, etc.)
//
// Design decision (waypoint array vs single point):
//   We use an ARRAY of waypoints, not a single point.  Production CMM
//   programs are intrinsically sequences (datum → primary touches →
//   features → reposition → next datum).  A single-point variant would
//   just duplicate probe_point; the array form lets cmm_scan emit a
//   complete inspection ticket and lets the planner reason about cycle
//   time, stylus reposition count, and per-point pass/fail.  Empty
//   measured_mm[] is allowed (pre-flight planning) — when present, the
//   array must have the same length as nominal_points_3d[].
//
// Input:
//   nominal_points_3d  — vector<gp_Pnt> nominal probe waypoints in world
//   measured_mm        — optional measured deviation along surface normal
//                        for each waypoint (signed, mm).  Empty ⇒ planning.
//   target_tolerance_mm— global acceptance band (e.g. 0.025 mm)
//   stylus_diameter_mm — ruby ball diameter (default 2 mm)
//
// DFM checks:
//   nominal_points_3d must not be empty
//   target_tolerance_mm must be > 0
//   if measured_mm non-empty: size must match nominal count
//   measured deltas above target_tolerance produce a WARNING per point
//   stylus_diameter_mm must be > 0
//
// Recognize:
//   Metadata-only operation — leaves no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cmm_scan {

constexpr const char* kSkillId = "cmm_scan";

struct Input
{
    std::vector<gp_Pnt>  nominal_points_3d;
    std::vector<double>  measured_mm;            // empty OR same size as nominal
    double               target_tolerance_mm = 0.025;
    double               stylus_diameter_mm  = 2.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cmm_scan
}  // namespace koocadcam::skill
