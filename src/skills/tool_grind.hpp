#pragma once
// @lat: [[engine/skills#tool_grind]]
//
// tool_grind — grind the cutting edge of a manufactured cutting tool
// (end mill, drill, insert) to the target rake / relief / edge-radius
// geometry.  The skill operates on a TOOL — the macroscopic WORKPIECE
// being processed by KooCADCAM is the tool blank itself.  As such the
// CAD geometry of the workpiece on the bench is NOT updated by this
// skill; instead the achieved edge geometry is captured in the
// signature for downstream traceability + CAM tool-library updates.
//
//             ┌────────────────┐         rake α / relief γ
//             │   blank        │   tool_grind   ┌─────────────┐
//             │   (cylinder)   │  ─────────▶   │  ground     │
//             │                │  α, γ, r_edge │  edge       │
//             └────────────────┘                └─────────────┘
//
// Signature pattern:
//   - pattern.kind                 = "tool_grind"
//   - pattern.tool_id              = <input>
//   - pattern.rake_angle_deg       = <input>
//   - pattern.relief_angle_deg     = <input>
//   - pattern.target_edge_radius_um= <input>
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   tool_id non-empty                       (DFM-INPUT error if not)
//   target_edge_radius_um > 0               (DFM-INPUT error if not)
//   rake_angle_deg in [-30, +30]            (DFM-TG-RAKE warning if outside)
//   relief_angle_deg in [0, 25]             (DFM-TG-RELIEF warning if outside)
//   target_edge_radius_um in [2, 50]        (DFM-TG-RADIUS info if outside)
//
// Synthesis:
//   NO geometric change to the bench workpiece.  Clone + stamp.
//
// Recognize:
//   Metadata replay only — tool-edge geometry lives in CAM tool library,
//   not in the workpiece B-rep.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tool_grind {

constexpr const char* kSkillId = "tool_grind";

struct Input
{
    std::string tool_id                 = "EM-6MM-4F-001";
    double      rake_angle_deg          = 10.0;     // typical 5..15
    double      relief_angle_deg        = 8.0;      // typical 6..12
    double      target_edge_radius_um   = 10.0;     // typical 5..20 um
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tool_grind
}  // namespace koocadcam::skill
