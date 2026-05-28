#pragma once
// @lat: [[engine/skills#friction_stir]]
//
// friction_stir — Friction Stir Welding (FSW).  A rotating, non-consumable
// tool plunges into the joint line and traverses; frictional heat plus
// severe plastic deformation forge a solid-state bond.  NO filler material
// is added.  Used for aluminum (most often), magnesium, and increasingly
// for high-strength steels in automotive battery trays and aerospace
// fuselage panels.
//
//   Side view — FSW signature on the finished part:
//                    ▼  rotating tool shoulder
//                  ┌──────┐
//                  └──────┘
//   ────────╲                ╱──────────       ← entry_face
//             ╲____________╱   ← shallow groove (typ. 0.1 – 0.3 mm)
//
// Geometric model:
//   Unlike all the OTHER welds in this family, friction stir REMOVES a
//   small amount of material — the surface is mechanically disturbed by
//   the tool shoulder, leaving a shallow groove of width
//   `tool_shoulder_dia_mm` and depth 0.1 – 0.3 mm.  We model that as a
//   subtractive Boolean using a thin stadium-shaped CUTTER (same shape
//   family as mill_slot but very shallow).
//
// Signature pattern:
//   - 2 half-cylinder ends (concave / inward)
//   - 2 long planar walls (recessed)
//   - 1 bottom planar face
//   - pattern.kind = "friction_stir"
//   - subtractive (depression rather than protrusion)
//
// DFM checks:
//   tool_shoulder_dia_mm ≥ 6.0  (commercial FSW pin tool floor)
//   tool_rotation_rpm    ∈ [200, 3000]
//   traverse_speed_mm_per_s > 0
//   depth_mm             ∈ [0.05, 0.5]
//   material            ∈ {aluminum, magnesium, steel} (INFO)
//
// Recognize:
//   Look for a SHALLOW stadium-shape DEPRESSION on a planar face — same
//   topology as mill_slot but with very small depth (< 0.5 mm) and tool
//   shoulder dia ≥ 6 mm.  Falls back to metadata if present.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace friction_stir {

constexpr const char* kSkillId = "friction_stir";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm                = 0.0;
    double      start_y_mm                = 0.0;
    double      end_x_mm                  = 0.0;
    double      end_y_mm                  = 0.0;
    double      tool_shoulder_dia_mm      = 12.0;  // typ. 10 – 25 mm
    double      tool_rotation_rpm         = 1200.0;
    double      traverse_speed_mm_per_s   = 5.0;
    double      depth_mm                  = 0.2;   // shallow trace (0.1 – 0.3 mm)
    std::string material                  = "aluminum";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace friction_stir
}  // namespace koocadcam::skill
