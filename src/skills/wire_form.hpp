#pragma once
// @lat: [[engine/skills#wire_form]]
//
// wire_form — bend a length of round wire into a 3D path on a CNC wire
// former.  The wire is fed straight and bent in sequence at programmed
// points around tooling pins to follow an arbitrary 3D polyline.  Used for
// staples, paperclips, harness retainers, automotive harness brackets,
// medical-device stylets, etc.
//
//                              ┌──┐
//             ───────────────┐ │  │
//                            │ │  │
//                            │ │  │       waypoint 4 ─►
//                            │ │  │       (turn-back)
//             ◄── waypoint 1 │ └──┘ waypoint 3
//                wire feed   └──────  waypoint 2
//
// Slice-1 representation:
//   The skill is METADATA-ONLY.  Sweeping a circular section along a
//   polyline produces a valid OCCT solid but with high face count and
//   fragility at sharp corners; for slice-1 we record the polyline and
//   wire diameter as signature data so renderers / FEA can rebuild on
//   demand.  Workpiece geometry passes through unchanged.
//
// Signature pattern:
//   - kind             = wire_form
//   - waypoints        = JSON array of (x, y, z) ∈ R³ ; ≥ 2 points
//   - wire_dia_mm      = wire diameter
//   - polyline_length_mm = total path length (≈ wire used)
//   - bend_count       = (waypoints.size() − 2) (interior corners)
//   - min_bend_radius_mm = smallest local turning radius along the polyline
//   - geometry_changed = false
//
// DFM checks:
//   DFM-INPUT          : ≥ 2 waypoints, wire_dia_mm > 0
//   DFM-WF-BEND-R      : implied min_bend_radius < 2 × wire_dia_mm — error
//                        (industrial minimum: bend radius ≥ 2 d to avoid
//                         cracking on outer fibre)
//   DFM-WF-COLLINEAR   : 3 collinear consecutive points → info (degenerate
//                        bend, machine wastes a programming step)
//
// Recognize:
//   Metadata-only.  A bent-wire path inside a static B-rep cannot be
//   distinguished from any other linear/curvilinear thin solid without
//   dedicated centreline extraction.  We replay from features().

#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wire_form {

constexpr const char* kSkillId = "wire_form";

struct Waypoint
{
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
};

struct Input
{
    // Polyline in world space; ≥ 2 waypoints.
    std::vector<Waypoint>  waypoints;

    // Wire diameter.
    double                 wire_dia_mm = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wire_form
}  // namespace koocadcam::skill
