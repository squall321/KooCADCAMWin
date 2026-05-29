#pragma once
// @lat: [[engine/skills#gear_blank_with_hub_step]]
//
// gear_blank_with_hub_step — COMPOUND macro that produces a stepped
// gear blank from cylindrical stock:
//
//   sub-cuts (3):
//     1. top-face turn-down to hub_dia                  (annular ring removal)
//     2. central through-bore                           (Ø bore_dia)
//     3. keyway in bore                                 (DIN 6885 rectangle)
//
// The hub step lets the gear be located against a shoulder during the
// downstream hob/grind operation.  Bore + keyway is the standard
// shaft-mount interface.
//
// DFM gates:
//   DFM-INPUT, DFM-002,
//   DFM-GEAR-HUB-DIA   hub_dia >= blank_dia (no step)
//   DFM-GEAR-WALL      (hub_dia - bore_dia)/2 < 3 mm (hub too thin)
//   DFM-GEAR-HUB-Z     hub_height >= thickness        (step deeper than blank)
//   DFM-KEYWAY-FIT     keyway_w < 1 mm OR > bore_dia
//   DFM-KEYWAY-DEPTH   keyway_h <= 0

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gear_blank_with_hub_step {

constexpr const char* kSkillId = "gear_blank_with_hub_step";

struct Input
{
    double blank_dia_mm    = 0.0;
    double blank_thick_mm  = 0.0;   // total axial thickness of the blank
    double hub_dia_mm      = 0.0;   // smaller diameter at the hub side
    double hub_height_mm   = 0.0;   // axial extent of the hub step (< blank_thick)
    double bore_dia_mm     = 0.0;
    double keyway_w_mm     = 0.0;   // 0 → skip keyway
    double keyway_h_mm     = 0.0;   // 0 → skip keyway
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gear_blank_with_hub_step
}  // namespace koocadcam::skill
