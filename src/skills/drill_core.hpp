#pragma once
// @lat: [[engine/skills#drill_core]]
//
// drill_core — mining exploration core-drilling record.  A rotary, percussive,
// or diamond drill string is driven into rock to recover (or destroy) a
// cylindrical core of known diameter and depth.  The workpiece (rock mass /
// orebody surrogate) is not modified geometrically by KooCADCAM — the
// macroscopic shape change is below the model's level of detail — but a
// signature is stamped so downstream process planning (assay scheduling,
// drill-rod inventory, mud handling) can replay it.
//
//                 │ rig
//                 ▼
//        ┌────────────────┐
//        │   drill_core   │   → recovered core (or chips)
//        │ hole_dia, depth│
//        │ method:        │
//        │  rotary |      │
//        │  percussive |  │
//        │  diamond       │
//        └────────────────┘
//
// Signature pattern:
//   pattern.kind             = "drill_core"
//   pattern.hole_dia_mm      = double
//   pattern.depth_m          = double
//   pattern.drill_method     = "rotary" | "percussive" | "diamond"
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             hole_dia_mm ≤ 0 or depth_m ≤ 0
//   DFM-DRILL-METHOD      drill_method not in {rotary, percussive, diamond}
//   DFM-CORE-DIA-HIGH     hole_dia_mm > 200 (industrial outsize, info)
//   DFM-CORE-DEPTH-HIGH   depth_m > 2000   (deep exploration, info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace drill_core {

constexpr const char* kSkillId = "drill_core";

struct Input
{
    double      hole_dia_mm  = 76.0;        // NQ/HQ wireline ~60–96 mm
    double      depth_m      = 100.0;
    std::string drill_method = "diamond";   // "rotary" | "percussive" | "diamond"
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drill_core
}  // namespace koocadcam::skill
