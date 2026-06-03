#pragma once
// @lat: [[engine/skills#intermittent_weld_bead]]
//
// intermittent_weld_bead — Replicated short fillet weld beads at intervals
// along a direction.  Builds N short boxes ("bead" + "gap" pattern) and
// FUSES them onto the workpiece.  Typical SolidWorks Weldments
// stitch-weld feature.
//
//      bead ── gap ── bead ── gap ── bead ── gap ── …
//      ▓▓▓▓░░░░▓▓▓▓░░░░▓▓▓▓░░░░▓▓▓▓
//
// Signature pattern:
//   pattern.kind             = "intermittent_weld_bead"
//   pattern.is_compound      = true
//   pattern.profile_kind     = "bead"
//   pattern.bead_count
//   pattern.derived_volume_mm3
//
// DFM:
//   DFM-INPUT      : bead_length > 0, bead_count > 0, gap_length ≥ 0,
//                    bead_height > 0, direction non-zero.

#include "Skill.hpp"

#include <array>

namespace koocadcam::skill {

class Workpiece;

namespace intermittent_weld_bead {

constexpr const char* kSkillId = "intermittent_weld_bead";

struct Input
{
    std::array<double, 3>  bead_start_xyz     = { 0.0, 0.0, 0.0 };
    std::array<double, 3>  bead_direction_xyz = { 1.0, 0.0, 0.0 };
    double                 bead_length_mm     = 25.0;
    double                 gap_length_mm      = 50.0;
    double                 bead_height_mm     = 5.0;
    int                    bead_count         = 4;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace intermittent_weld_bead
}  // namespace koocadcam::skill
