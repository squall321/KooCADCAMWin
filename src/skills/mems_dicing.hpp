#pragma once
// @lat: [[engine/skills#mems_dicing]]
//
// mems_dicing — singulate MEMS / semiconductor dies from a processed wafer
// using a precision saw or stealth-laser.  The wafer is mounted on dicing
// tape; saw kerfs run along scribe streets to release individual dies.
// In our process planner we treat the wafer B-rep as unchanged (the streets
// and dies are sub-OCCT entities) and stamp the dicing recipe so downstream
// pick-and-place / die_attach picks up kerf width and die footprint.
//
//        wafer ──┬───┬───┬───┬───┬───┬──
//                │die│die│die│die│die│        ← die_size_mm pitch
//                ├───┼───┼───┼───┼───┤
//                │die│die│die│die│die│
//                ──┴───┴───┴───┴───┴───
//                        ↑
//                   kerf_um (street width)
//
// Signature pattern:
//   - pattern.kind            = "mems_dicing"
//   - pattern.kerf_um         = saw / laser cut width in micrometres
//   - pattern.die_size_mm     = square die edge length
//   - pattern.blade_type      = "diamond_resin" | "Ni_bonded" | "stealth_laser"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   kerf_um      ∈ [10, 200]                       (DFM-KERF error if outside)
//   die_size_mm  ∈ [0.3, 30.0]                     (DFM-DIE-SIZE error)
//   blade_type   ∈ {diamond_resin, Ni_bonded,
//                   stealth_laser}                  (DFM-BLADE info)
//
// Synthesis: NO geometric change — clone shape + material, stamp signature.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mems_dicing {

constexpr const char* kSkillId = "mems_dicing";

struct Input
{
    double      kerf_um     = 50.0;
    double      die_size_mm = 3.0;
    std::string blade_type  = "diamond_resin";   // "diamond_resin" | "Ni_bonded" | "stealth_laser"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mems_dicing
}  // namespace koocadcam::skill
