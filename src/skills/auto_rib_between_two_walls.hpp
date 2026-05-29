#pragma once
// @lat: [[engine/skills#auto_rib_between_two_walls]]
//
// auto_rib_between_two_walls — CONTEXT-AWARE compound macro that fuses a
// rectangular rib that BRIDGES two specified wall faces.  The length of
// the rib is auto-detected from the geometric centres of the two faces,
// so the caller doesn't have to compute it.
//
// Sub-features (subfeature_count = 3):
//   1. main rectangular rib block fused between wall centres   [fuse]
//   2. small chamfer fillet onto wall A (chamfer cut)          [cut]
//   3. small chamfer fillet onto wall B (chamfer cut)          [cut]
//
// Lookup table — Plastic rib spec (% wall thickness) per BASF moulding
// design guide:
//   "thin"    → rib_thick = 0.50 × wall_thick (≤ sink mark)
//   "standard"→ rib_thick = 0.60 × wall_thick (recommended default)
//   "thick"   → rib_thick = 0.75 × wall_thick (use only with cooling)
//
// Context detection:
//   - Read centres of wall A and wall B from the workpiece (faceCenter).
//   - Use the midpoint as rib origin, length = distance between centres.
//   - Rib height = min(face A area, face B area) / rib_thick OR a sane
//     default fraction of the workpiece bbox z range.
//
// DFM:
//   DFM-INPUT       : missing face ids / rib_thick.
//   DFM-RIB-SPEC    : unknown style key.
//   DFM-RIB-RATIO   : rib_thick > 0.75 × min(face_dim) (sink-mark risk).
//   DFM-RIB-AR      : aspect ratio rib_length / rib_thick > 40
//                     (warpage risk per BASF chart).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace auto_rib_between_two_walls {

constexpr const char* kSkillId = "auto_rib_between_two_walls";

struct Input
{
    int         wall_A_face_id = -1;
    int         wall_B_face_id = -1;
    double      rib_thick_mm   = 1.5;
    double      rib_height_mm  = 0.0;      // 0 → auto from context (workpiece bbox z * 0.6)
    std::string rib_style      = "standard";  // "thin"/"standard"/"thick"
    double      wall_thick_mm  = 2.5;      // hosting wall thickness reference
};

SkillOutput   apply   (const Workpiece& wp, const Input& in);
DFMReport     validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace auto_rib_between_two_walls
}  // namespace koocadcam::skill
