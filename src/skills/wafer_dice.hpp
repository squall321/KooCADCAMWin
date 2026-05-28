#pragma once
// @lat: [[engine/skills#wafer_dice]]
//
// wafer_dice — semiconductor back-end singulation step.  A processed wafer
// is cut along orthogonal streets to release individual dies.  Modelled as
// a METADATA-ONLY skill: we record the wafer/die geometry but do not
// perform the OCCT Boolean cut on the actual workpiece (which would
// require ~ wafer_dia / (die_pitch + kerf) operations and is out of
// scope for the planning record).
//
//        ┌─────────────────────┐                ┌──┬──┬──┬──┐
//        │       wafer         │   wafer_dice   │  │  │  │  │
//        │   (Ø150 mm say)     │  ────────────▶ │  │  │  │  │
//        │                     │   (die grid)   ├──┼──┼──┼──┤
//        └─────────────────────┘                │  │  │  │  │
//                                               └──┴──┴──┴──┘
//          shape preserved in planning record;  signature captures die grid
//
// Signature pattern:
//   - pattern.kind            = "wafer_dice"
//   - pattern.wafer_dia_mm    = <input>
//   - pattern.die_size_x_mm   = <input>
//   - pattern.die_size_y_mm   = <input>
//   - pattern.kerf_um         = <input>
//   - pattern.die_count_est   = floor(usable_area / (die_x · die_y))
//   - pattern.is_separation   = true
//
// DFM checks:
//   wafer_dia_mm  > 0  and  ≤ 450 mm (450 mm is the largest production)
//   die_size_x_mm > 0  and  ≤ wafer_dia
//   die_size_y_mm > 0  and  ≤ wafer_dia
//   kerf_um       > 0  and  < die_size (otherwise we kerf through whole die)
//
// Recognize:
//   Metadata-only.  No geometric trace.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wafer_dice {

constexpr const char* kSkillId = "wafer_dice";

struct Input
{
    double  wafer_dia_mm  = 150.0;   // 100 / 150 / 200 / 300 / 450 typical
    double  die_size_x_mm = 5.0;
    double  die_size_y_mm = 5.0;
    double  kerf_um       = 30.0;    // dicing-saw blade width
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wafer_dice
}  // namespace koocadcam::skill
