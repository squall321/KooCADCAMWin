#pragma once
// @lat: [[engine/skills#snap_assembly]]
//
// snap_assembly — assembly-level snap connection between two parts.  This
// workpiece carries the female side: a small rectangular CAVITY (subtractive
// pocket) sized to accept a matching tab on the mating part.  The tab itself
// is recorded only as metadata so the assembly planner can size the partner
// part's protrusion to suit.
//
//   Side view (looking along the tab's WIDTH direction):
//
//                                  ───────► insertion_axis_xy (into cavity)
//
//                           ┌─ entry_face (planar)
//                           ▼
//                   ────────┬────────────┐
//                           │ ░░░░░░░░░░ │  ← cavity (this workpiece)
//                           │ ░░░░░░░░░░ │     cavity_w × cavity_l × cavity_d
//                           │ ░░░░░░░░░░ │
//                   ────────┴────────────┴────────
//
//                              │ tab on partner
//                              │ (metadata only)
//                              ▼
//
// Signature pattern:
//   - 4 planar wall faces  (cavity sides) — 2 along length, 2 along width
//   - 1 planar bottom face
//   - 4 sharp planar edges at the entry rim
//   - kind = "snap_assembly", carries cavity_w / cavity_l / cavity_d +
//     tab_engage / partner_part metadata
//
// DFM checks:
//   DFM-SA-WIDTH    : cavity_width_mm  ≥ 1.0 mm
//   DFM-SA-LENGTH   : cavity_length_mm ≥ 1.0 mm
//   DFM-SA-DEPTH    : cavity_depth_mm  ∈ [0.5, 10.0] mm
//   DFM-SA-TAB      : tab_engage_mm < cavity_depth_mm (tab must fit)
//   DFM-SA-WALL     : cavity is wider than the wall, but caller-side concern
//                     (we only enforce the cavity-side geometry here).
//
// Recognize:
//   History replay (confidence 1.0).  Geometric fallback iterates for a
//   small rectangular pocket whose footprint area is < 50 mm² and whose
//   depth is in [0.5, 10] mm; reports confidence 0.5 because a plain
//   mill_rect_pocket of the same shape is indistinguishable without metadata.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace snap_assembly {

constexpr const char* kSkillId = "snap_assembly";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm     = 0.0;       // cavity centre on entry face
    double      position_y_mm     = 0.0;
    gp_Dir      insertion_axis_xy { 1.0, 0.0, 0.0 };  // tab insertion direction in XY plane
    double      cavity_length_mm  = 0.0;       // along insertion_axis_xy
    double      cavity_width_mm   = 0.0;       // perpendicular to insertion_axis_xy
    double      cavity_depth_mm   = 0.0;       // into the entry_face
    double      tab_engage_mm     = 0.0;       // how deep the tab sits in the cavity
    std::string partner_part      = "";        // mating part name (tab side)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace snap_assembly
}  // namespace koocadcam::skill
