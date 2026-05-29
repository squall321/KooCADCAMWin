#pragma once
// @lat: [[engine/skills#food_mixing]]
//
// food_mixing — bulk mixing of food ingredients in a process vessel.
//
//   Combine dry / wet ingredients in a ribbon, paddle, planetary, or
//   high-shear mixer to produce a homogeneous bulk batch.  Macroscopic
//   geometry of the vessel/batch is treated as unchanged for the CAD/CAM
//   layer; the operation is a process-only stamp that records WHAT was
//   mixed, in WHICH MIXER, for HOW LONG, at WHAT SCALE.
//
//        ┌─────────────┐                ┌─────────────┐
//        │ feed        │   food_mixing  │ bulk batch  │
//        │  streams    │  ──────────▶  │  (homog.)   │
//        └─────────────┘                └─────────────┘
//        geometry unchanged ; ingredient distribution → uniform
//
// Signature pattern:
//   pattern.kind            = "food_mixing"
//   pattern.mixer_type      = "ribbon" | "paddle" | "planetary" |
//                             "high_shear" | "double_cone"
//   pattern.batch_kg        = double  (load mass in kilograms)
//   pattern.mix_time_min    = double  (minutes)
//   pattern.process_family  = "food_processing"
//   pattern.geometric_change= false
//
// DFM:
//   DFM-INPUT               mixer_type empty / unknown
//   DFM-MIX-BATCH           batch_kg outside (0, 10000] kg
//   DFM-MIX-TIME            mix_time_min outside (0, 240] minutes
//   DFM-MIX-TIME-SHORT      mix_time_min < 1 min (info — homogeneity risk)
//
// Recognize: metadata-only.  Bulk mixing leaves no analytic B-Rep trace.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace food_mixing {

constexpr const char* kSkillId = "food_mixing";

struct Input
{
    std::string  mixer_type     = "ribbon";   // see header
    double       batch_kg       = 100.0;       // kg
    double       mix_time_min   = 15.0;        // minutes
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace food_mixing
}  // namespace koocadcam::skill
