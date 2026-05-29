#pragma once
// @lat: [[engine/skills#ratchet_pawl_set]]
//
// ratchet_pawl_set — REAL compound feature producing a ratchet wheel:
//
//   sub-cuts (N + 1):
//     1. central through bore
//     2..N+1. N asymmetric tooth-gap cutouts equispaced around the wheel.
//             Each gap has a STEEP back flank (drive) and SHALLOW front
//             flank (slide).  Ratchet teeth allow rotation in one direction
//             while a pawl blocks the other.
//
//   The pawl is NOT cut into the wheel; we return a "side note" pawl shape
//   description in the signature.extra (a small box centered offset from
//   the wheel) — actual pawl assembly is a downstream concern.
//
// Tooth profile (ANSI / general practice):
//   - steep flank: 80-85° from tangent (effectively radial, locks)
//   - shallow flank: 30-45° from tangent (slides under pawl on free rotation)
//   - tooth depth: 0.3 * module * pitch_dia  (rule of thumb)
//
// DFM gates:
//   DFM-RATCHET-N        N < 6 or N > 60
//   DFM-RATCHET-DEPTH    tooth_depth_mm <= 0 or > 0.3 * pitch_dia
//   DFM-RATCHET-STEEP    steep_flank_deg outside [70, 90]
//   DFM-RATCHET-SHALLOW  shallow_flank_deg outside [20, 60]
//   DFM-RATCHET-BORE     bore collides with root circle

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ratchet_pawl_set {

constexpr const char* kSkillId = "ratchet_pawl_set";

struct Input
{
    double  wheel_dia_mm        = 0.0;     // outer diameter (top of tooth)
    double  wheel_thick_mm      = 0.0;
    int     num_teeth           = 0;
    double  tooth_depth_mm      = 0.0;     // radial gap depth
    double  steep_flank_deg     = 85.0;    // drive flank
    double  shallow_flank_deg   = 30.0;    // slide flank
    double  bore_dia_mm         = 0.0;
    bool    ccw_locking         = true;    // direction blocked by pawl
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ratchet_pawl_set
}  // namespace koocadcam::skill
