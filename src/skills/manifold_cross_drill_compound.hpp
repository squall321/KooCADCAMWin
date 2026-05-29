#pragma once
// @lat: [[engine/skills#manifold_cross_drill_compound]]
//
// manifold_cross_drill_compound — COMPOUND MACRO that machines two
// perpendicular drilled passages intersecting inside a block plus a plug-
// thread counterbore on each end face for sealing.
//
//                          +Z
//                           │
//                           │
//   ╔═══════════════════════╪═══════════════════════╗
//   ║                       │                       ║
//   ║   ┌──┐                │              ┌──┐    ║
//   ║   │CB│====drill1======│============│CB│=== +X
//   ║   └──┘                │              └──┘    ║
//   ║                       │                       ║
//   ║                       │                       ║
//   ╠═══════════════════════O═══════════════════════╣
//   ║                       │                       ║
//   ║                  ┌─drill2─┐                   ║
//   ║                  └────────┘                   ║
//   ╚═══════════════════════│═══════════════════════╝
//
// This is the standard hydraulic / pneumatic manifold cross-drill: two
// passages cross in the block interior so flow can be tee'd between the
// four open ports.  Each port gets a counterbore at its mouth to accept a
// pipe plug (typically NPT or BSPT) that seals unused branches.
//
// Sub-feature chain (4 primitive cuts):
//   1. Drill #1 along `drill1_axis` (through-hole across the block).
//   2. Drill #2 along `drill2_axis` (perpendicular through-hole).
//   3. Counterbore at the +end of drill1_axis (plug seat).
//   4. Counterbore at the +end of drill2_axis (plug seat).
//
// (We model exactly 4 cuts because the four end-face plug seats are
//  conventionally counterbored only at the two "active" ends — the
//  opposite ends remain open as the actual inlet/outlet ports.  Bumping
//  subfeature_count up if symmetric plugs are needed is a future option.)
//
// DFM gates:
//   DFM-INPUT             : non-positive sizes / non-perpendicular axes.
//   DFM-MANIFOLD-CROSS    : drill_dia too small for cross intersection (< 2 mm).
//   DFM-002               : individual drill diameter < 0.8 mm.
//   DFM-MANIFOLD-PLUG     : plug_M unknown / pilot collision with drill bore.
//
// Recognize:
//   This is a compound metadata feature — recognize() reads `wp.features()`
//   for an existing manifold_cross_drill_compound signature and replays
//   the input verbatim (confidence 1.0).  Pure-geometry recognition is
//   not attempted (two intersecting through-holes look identical to two
//   independent drills downstream).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace manifold_cross_drill_compound {

constexpr const char* kSkillId = "manifold_cross_drill_compound";

struct Input
{
    FaceDatum   block_face;                                     // reference face for datum
    gp_Dir      drill1_axis        { 1.0, 0.0, 0.0 };           // first passage (any unit dir)
    gp_Dir      drill2_axis        { 0.0, 1.0, 0.0 };           // perpendicular to drill1_axis
    double      drill1_origin_x_mm = 0.0;                       // common XY mid (block center)
    double      drill1_origin_y_mm = 0.0;
    double      drill1_origin_z_mm = 0.0;
    double      drill_dia_mm       = 6.0;                       // passage diameter (both)
    std::string plug_M;                                         // "M6", "M8", "M10", "M12"
    double      plug_seat_depth_mm = 4.0;                       // depth of counterbore seat
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace manifold_cross_drill_compound
}  // namespace koocadcam::skill
