#pragma once
// @lat: [[engine/skills#snap_fit]]
//
// snap_fit — L-shaped flexible hook used for clip-on assemblies between two
// injection-moulded parts (e.g. battery cover, headphone case lid).
//
//  Side view (looking along the rib's width direction):
//
//                     ◄── hook_depth ──►
//                                       ┌──────┐  ▲ hook_height
//                                       │ hook │  ▼
//                  ┌────────────────────┴──────┘
//                  │  cantilever beam (length)   │
//                  │   wall_thickness            │
//      ────────────┴─────────────────────────────┴──────  entry_face
//                  ◄────── length ──────►
//
// Plan view: a rectangle of length × width, plus a perpendicular "hook lip"
// at one end (depth × width).
//
// The geometry is a fuseMany of two boxes:
//   1) Cantilever beam: length × width × wall_thickness, extruded along
//      `outward` from entry_face plane.
//   2) Hook lip: hook_depth × width × hook_height, sitting at the far end
//      of the beam, extruded along outward starting at z = wall_thickness
//      (i.e. on top of the beam's top face).  The lip extends past the
//      beam end in the +direction_xy by hook_depth.
//
// Signature pattern:
//   - L-shaped protrusion: 2 perpendicular planar faces meeting in a 90°
//     corner at the hook step.
//   - Identifiable by aspect (length / wall_thickness > 3) and the presence
//     of a step.
//
// DFM:
//   DFM-SNAP-WALL    : wall_thickness ≥ 0.8 mm
//   DFM-SNAP-HOOK    : hook_depth ≤ hook_height (engagement geometry)
//   DFM-SNAP-LENGTH  : length ≥ wall_thickness × 5 (else not flexible)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace snap_fit {

constexpr const char* kSkillId = "snap_fit";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm         = 0.0;
    double      start_y_mm         = 0.0;
    gp_Dir      direction_xy       { 1.0, 0.0, 0.0 };  // unit vector in XY plane
    double      length_mm          = 0.0;
    double      hook_height_mm     = 0.0;
    double      hook_depth_mm      = 0.0;
    double      width_mm           = 0.0;
    double      wall_thickness_mm  = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace snap_fit
}  // namespace koocadcam::skill
