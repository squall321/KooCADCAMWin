#pragma once
// @lat: [[engine/skills#penetrant_weld]]
//
// penetrant_weld — liquid dye-penetrant inspection (PT) of a weld, per
// AWS D1.x acceptance.  Catches surface-breaking discontinuities on
// non-ferromagnetic materials (stainless, aluminium, etc.).
// Metadata-only.
//
//   1) clean        2) penetrant       3) developer  → red indications on white
//
// Sensitivity classes (ASTM E1417):
//   "level_1"  ultra-low   "level_2"  low    "level_3"  medium
//   "level_4"  high        "level_5"  ultra-high
//
// Pattern:
//   pattern.kind               = "penetrant_weld"
//   pattern.weld_id            = <input>
//   pattern.sensitivity_class  = <input>
//   pattern.accept             = <input bool>
//   pattern.geometry_changed   = false
//
// DFM:
//   DFM-INPUT                weld_id empty
//   DFM-NDT-SENS             sensitivity_class not in level_1..level_5

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace penetrant_weld {

constexpr const char* kSkillId = "penetrant_weld";

struct Input
{
    std::string  weld_id;
    std::string  sensitivity_class = "level_3";  // level_1 .. level_5
    bool         accept            = true;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace penetrant_weld
}  // namespace koocadcam::skill
