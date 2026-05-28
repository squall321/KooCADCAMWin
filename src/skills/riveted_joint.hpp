#pragma once
// @lat: [[engine/skills#riveted_joint]]
//
// riveted_joint — through-hole + a small rivet-head bump on each side of the
// workpiece.  Models the deformed state of a set rivet: shank fills the hole,
// each end is upset into a low rounded disc protruding from the surface.
//
//                         ─── head_dia_mm ───
//                         ┌──────────────┐   ▲
//                         │              │   │ head_height_mm
//                ─────────┴──────────────┴───┴──────  ← entry_face
//                         │   ▼ axis     │
//                         │              │
//                         │  rivet shank │  ← cylindrical hole
//                         │              │
//                ─────────┬──────────────┬───────────  ← exit face
//                         │              │   ▲
//                         └──────────────┘   ▼ head_height_mm
//                         ────── head_dia_mm ──
//
// Geometry steps (apply):
//   1. CUT the through-hole (cylinder along axis_dir, full bbox length).
//   2. FUSE a low cylindrical bump on each side of the workpiece (one at the
//      entry face, one at the exit face) sized at head_dia_mm × head_height_mm.
//
// Signature pattern:
//   - 3 cylindrical faces: 1 bore wall + 2 head side walls
//   - 4 circular edges on the bore + heads (entry/exit + head-top × 2)
//   - 2 small planar discs on top of each head
//   - kind = "riveted_joint" carrying rivet_dia / head_dia / head_height
//
// DFM checks:
//   DFM-RJ-DIA       : rivet_dia_mm ≥ 1.6 mm     (smallest standard)
//   DFM-RJ-HEAD-RATIO: head_dia_mm ≥ 1.5 × rivet_dia_mm
//   DFM-RJ-HEAD-H    : head_height_mm ∈ [0.3, head_dia_mm × 0.5]
//
// Recognize:
//   History replay (confidence 1.0).  Geometric fallback returns nothing —
//   the deformed rivet topology is ambiguous after a single Boolean fuse
//   (the rivet head can be confused with a spot_weld bead).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace riveted_joint {

constexpr const char* kSkillId = "riveted_joint";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    double      rivet_dia_mm   = 0.0;   // shank diameter (≈ bore size)
    double      head_dia_mm    = 0.0;   // upset head diameter (typ. 1.5 – 2.0 × shank)
    double      head_height_mm = 0.0;   // height of upset head above surface
    std::string material       = "aluminum";
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace riveted_joint
}  // namespace koocadcam::skill
