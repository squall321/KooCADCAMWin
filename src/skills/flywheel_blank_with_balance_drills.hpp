#pragma once
// @lat: [[engine/skills#flywheel_blank_with_balance_drills]]
//
// flywheel_blank_with_balance_drills — COMPOUND macro that finishes a
// flywheel blank by drilling balance-correction holes at N equispaced
// radial positions:
//
//   sub-cuts (N + 1):
//     1.   central bore                       (Ø bore_dia, through)
//     2…N+1. N radial balance-drill holes     (Ø drill_dia × drill_depth,
//                                              at radius hole_radius_mm)
//
// Real flywheel balancing drills holes at angles determined by spin-test;
// for synthesis we lay them down at uniform spacing as the geometric
// fingerprint.  Real-world recognize() would re-discover the drill pattern.
//
// DFM gates (ISO 1940 balance grade dimensional sanity):
//   DFM-INPUT, DFM-002
//   DFM-FLYWHEEL-COUNT     balance_drill_count < 2 or > 24
//   DFM-FLYWHEEL-RADIUS    hole_radius_mm + drill_r >= outer_dia / 2 (drill exits OD)
//   DFM-FLYWHEEL-DEPTH     drill_depth >= thickness (drill exits the other face)
//   DFM-FLYWHEEL-BORE      bore_dia < 0.8 mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace flywheel_blank_with_balance_drills {

constexpr const char* kSkillId = "flywheel_blank_with_balance_drills";

struct Input
{
    double outer_dia_mm           = 0.0;
    double thickness_mm           = 0.0;
    double bore_dia_mm            = 0.0;
    int    balance_drill_count    = 6;     // typically 6 / 8 / 12
    double balance_drill_dia_mm   = 6.0;
    double balance_drill_depth_mm = 5.0;
    double hole_radius_mm         = 0.0;   // radial position; 0 → auto = 0.85·R_out
    double angle_offset_deg       = 0.0;   // angular offset of hole #0
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace flywheel_blank_with_balance_drills
}  // namespace koocadcam::skill
