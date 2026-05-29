#pragma once
// @lat: [[engine/skills#spiral_back_up_ring_groove]]
//
// spiral_back_up_ring_groove — COMPOUND MACRO: O-ring groove + adjacent
// back-up ring slot for high-pressure / anti-extrusion service per
// Parker O-ring Handbook (section 5-1, back-up rings).
//
//   ┌────────────────────────────────┐  R_outer (stock outer)
//   │                                │
//   ────┐                       ┌──────  ← z = pos + W/2
//       │   ┌───┐   ┌──────┐    │       O-ring groove (depth G)
//       │   │   │   │      │    │       backup-ring slot (depth G_b = G)
//       │   │   │   │      │    │       width W_b ≈ 0.5·W typical
//       │   └───┘   └──────┘    │
//   ────┘                       └──────  ← z = pos - W/2
//                ↑ pos    ↑ pos + offset
//
// Sub-feature chain (geometric):
//   1. PRIMARY O-RING GROOVE — annular ring of width W_oring × depth G,
//      centered at position_z_mm.
//   2. BACK-UP RING SLOT — additional annular ring of width W_b × depth G_b
//      adjacent to (downstream of) the O-ring groove.  Cut as a separate
//      annular ring after the primary groove using fuse_first_then_cut.
//   3. INTERSEPTUM WALL — the small wall between O-ring and back-up ring
//      grooves is preserved (NOT a primitive cut — implicit topology).
//
// Per Parker O-ring Handbook Section 5-1:
//   - Back-up ring width W_b is typically equal to or slightly less than
//     the O-ring CS (0.5 × W_oring is a good rule of thumb for spiral
//     back-up rings).
//   - Back-up ring goes on the LOW-pressure side of the O-ring (downstream).
//   - For ONE-sided back-up: 1 slot.  For TWO-sided (bidirectional pressure):
//     2 slots, one on each side.  This skill cuts ONE slot (configurable
//     position via `position` direction in the input — "downstream" =
//     +ΔZ from O-ring center).
//
// Signature pattern: kind = "spiral_back_up_ring_groove", is_compound = true,
// subfeature_count = 2 (the two annular rings).
//
// DFM gates:
//   - ring_size must be in the AS568 lookup.
//   - bore_dia ≥ 2·G + 2.0 mm (need wall margin).
//   - groove combined width must fit within the stock Z extent.
//   - position must be one of "downstream" / "upstream" (string).

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spiral_back_up_ring_groove {

constexpr const char* kSkillId = "spiral_back_up_ring_groove";

struct Input
{
    double      bore_or_shaft_dia_mm = 0.0;    // stock outer diameter
    double      position_z_mm        = 0.0;    // O-ring groove center Z
    std::string ring_size;                     // AS568 dash code, e.g. "-210"
    std::string position             = "downstream";  // "downstream" | "upstream"
};

double crossSectionFor(const std::string& as568_dash);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spiral_back_up_ring_groove
}  // namespace koocadcam::skill
