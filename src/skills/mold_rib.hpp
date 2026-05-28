#pragma once
// @lat: [[engine/skills#mold_rib]]
//
// mold_rib — add a thin reinforcing wall perpendicular to a planar face for
// stiffness in injection-moulded parts.
//
//                                       ▲ height_mm
//                                       │
//                          ┌──────────┐ │
//                          │   rib    │ ↓                   ←─ thickness_mm
//                          │ (length) │
//                          │          │
//      ──────────┬─────────┴──────────┴──────────┬─────────  entry_face
//                                                            (planar — base)
//                       ◄────── length ──────►
//                   (start_x,y) → (end_x,y) direction
//
// A rib is a "wall" between two XY end-points on the entry face, extruded
// upward by `height_mm`.  Its in-plane footprint is a thin rectangle of
// (length × thickness).  Optionally tapered with `draft_angle_deg` so the
// rib pulls cleanly from the mould tool (cone-frustum-style extrusion).
//
// Signature pattern: a thin elongated additive box whose plan-view aspect
// ratio (length / thickness) is > 3.  Recognition tries to identify a
// thin Z-extruded protrusion on top of a base planar face.
//
// DFM:
//   DFM-RIB-MIN-THICK  : thickness ≥ 0.6 mm (injection moldable minimum)
//   DFM-RIB-SINK       : thickness should be ≤ 0.6 × wall_thickness to
//                        avoid sink marks on the opposite "show" face.
//                        wall_thickness is not directly known to the skill,
//                        so this is reported as info/warn against a default
//                        baseline of 1.6 mm if no surrounding context.
//   DFM-RIB-STAB       : height ≤ 3 × thickness (otherwise unstable).
//   DFM-RIB-DRAFT      : draft_angle ≥ 0.5 deg (mould release).

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mold_rib {

constexpr const char* kSkillId = "mold_rib";

struct Input
{
    FaceDatum   entry_face;                 // planar base face the rib sits on
    double      start_x_mm      = 0.0;      // rib axis start point (world XY)
    double      start_y_mm      = 0.0;
    double      end_x_mm        = 0.0;      // rib axis end point (world XY)
    double      end_y_mm        = 0.0;
    double      height_mm       = 0.0;      // extrusion length perpendicular to entry_face
    double      thickness_mm    = 0.0;      // rib width in plan view
    double      draft_angle_deg = 1.0;      // 0 = no draft (parallel walls)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mold_rib
}  // namespace koocadcam::skill
