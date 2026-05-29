#pragma once
// @lat: [[engine/skills#cooking_op]]
//
// cooking_op — thermal cooking of food product (steam, bake, fry, sterilize).
//
//   Apply controlled heat to raise the product to a target INTERNAL
//   temperature for food-safety lethality and sensory development.  The
//   CAD/CAM layer treats geometry as unchanged; the operation records the
//   thermal recipe so downstream HACCP, traceability, and process planners
//   can verify lethality (Fo, Pt, etc.) was achieved.
//
//        ┌─────────────┐                ┌─────────────┐
//        │ raw product │   cooking_op   │ cooked      │
//        │  (T_amb)    │  ──────────▶  │  (T_int*)   │
//        └─────────────┘                └─────────────┘
//        geometry unchanged ; core temperature → target_internal_temp_c
//
// Signature pattern:
//   pattern.kind                    = "cooking_op"
//   pattern.method                  = "steam" | "bake" | "fry" | "sterilize"
//   pattern.temp_c                  = double (chamber / medium temperature)
//   pattern.time_min                = double (hold minutes)
//   pattern.target_internal_temp_c  = double (core lethality target)
//   pattern.geometric_change        = false
//
// DFM:
//   DFM-INPUT             method empty / unknown
//   DFM-COOK-TEMP         temp_c outside (0, 300] °C
//   DFM-COOK-TIME         time_min outside (0, 600] minutes
//   DFM-COOK-INTERNAL     target_internal_temp_c outside (0, 200] °C
//   DFM-COOK-LETHALITY    target_internal_temp_c > temp_c (info — cannot
//                         achieve core temp above ambient/chamber)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cooking_op {

constexpr const char* kSkillId = "cooking_op";

struct Input
{
    std::string  method                  = "bake";   // see header
    double       temp_c                  = 180.0;     // chamber/medium °C
    double       time_min                = 30.0;      // minutes
    double       target_internal_temp_c  = 75.0;      // core °C (safety)
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace cooking_op
}  // namespace koocadcam::skill
