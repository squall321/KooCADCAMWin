#pragma once
// @lat: [[engine/skills#edge_prep_hone]]
//
// edge_prep_hone — micro-honing of the cutting edge of a manufactured
// cutting tool (insert / end mill / drill).  After grind, a tool has a
// sharp but FRAGILE edge.  Edge prep — usually drag-finishing or brush
// honing — converts the sharp edge to a controlled micro-radius (5..40 um)
// that resists chipping during cut.  Asymmetric (waterfall-shaped) edges
// are common for milling inserts (rake side larger than flank side).
//
//     before:    ─┐               after:    ─╮
//                 │ knife edge              ╮ ╲   honed micro-radius
//                 │  (prone to chip)         ╲ ╲  (10..30 um typical)
//                                              ╲
//
// Signature pattern:
//   - pattern.kind              = "edge_prep_hone"
//   - pattern.tool_id           = <input>
//   - pattern.target_radius_um  = <input>
//   - pattern.asymmetry_factor  = <input>      (rake-side r / flank-side r)
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   tool_id non-empty                       (DFM-INPUT error if not)
//   target_radius_um > 0                    (DFM-INPUT error if not)
//   target_radius_um in [3, 40]             (DFM-EP-RADIUS info if outside)
//   asymmetry_factor in [0.5, 3.0]          (DFM-EP-ASYM warning if outside)
//
// Synthesis:
//   NO geometric change to bench workpiece.  Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace edge_prep_hone {

constexpr const char* kSkillId = "edge_prep_hone";

struct Input
{
    std::string tool_id            = "IN-CNMG120408-001";
    double      target_radius_um   = 15.0;     // 5..30 typical
    double      asymmetry_factor   = 1.5;      // 1.0 symmetric; >1 waterfall
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace edge_prep_hone
}  // namespace koocadcam::skill
