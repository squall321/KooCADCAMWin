#pragma once
// @lat: [[engine/skills#progressive_die_stage]]
//
// progressive_die_stage — METADATA-ONLY skill that records a stage marker
// inside a progressive die strip.  Geometry is NOT modified: this skill
// only stamps an annotation onto the workpiece's feature history that
// process-planning consumers (CAPP, tooling layout, strip-design tools)
// can read to know which stage of a multi-station die the current
// workpiece corresponds to.
//
// Used in tandem with `blanking`, `piercing`, `bend`, `flange`, etc., to
// declare:
//   "this workpiece state corresponds to stage 3 of an 8-station die,
//    feeding a 25 mm-pitch coil strip."
//
// Signature pattern:
//   - kind                = "progressive_die_stage"
//   - stage_index         = N (1-based)
//   - station_position    = "blank" | "pierce" | "bend" | "form" | "trim" | "eject" | "idle"
//   - material_strip_pitch_mm
//   - geometry_changed    = false
//
// DFM checks:
//   DFM-INPUT             : stage_index ≥ 1
//   DFM-PDS-PITCH         : material_strip_pitch_mm ∈ (0, 200] mm typical
//
// Recognize:
//   Pure metadata replay — geometry contains no record of stage assignment.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace progressive_die_stage {

constexpr const char* kSkillId = "progressive_die_stage";

struct Input
{
    int          stage_index               = 1;
    std::string  station_position          = "blank";
    double       material_strip_pitch_mm   = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace progressive_die_stage
}  // namespace koocadcam::skill
