#pragma once
// @lat: [[engine/skills#sand_wash]]
//
// sand_wash — aggregate washing record: water-jet rinse of crushed sand
// to remove silt/clay fines and to drive the gradation toward a target
// fineness modulus (FM, ASTM C136).  Macroscopic stock geometry is
// unchanged at the CAD level — this is a batch-process step that adjusts
// the granular property metadata.
//
// Signature pattern:
//   - pattern.kind                   = "sand_wash"
//   - pattern.throughput_kg_h        = input
//   - pattern.final_fineness_modulus = input
//   - pattern.water_l_kg             = wash water consumption per kg
//   - pattern.water_total_l_h        = throughput × water_l_kg (derived)
//   - pattern.geometry_changed       = false
//
// DFM checks:
//   DFM-INPUT             throughput_kg_h ≤ 0 (error)
//   DFM-INPUT             water_l_kg < 0 (error)
//   DFM-SAND-FM-RANGE     final_fineness_modulus outside [2.0, 3.5] (info)
//                         (per ASTM C33 concrete sand, FM 2.3–3.1 is typical)
//   DFM-SAND-WATER-LOW    water_l_kg < 0.3 (info — under-washed, fines remain)
//   DFM-SAND-WATER-HIGH   water_l_kg > 5.0 (info — wasteful, recycle water)
//
// Synthesis:
//   NO geometric change.  Clone and stamp.
//
// Recognize:
//   Metadata-only — washing leaves no B-rep trace.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sand_wash {

constexpr const char* kSkillId = "sand_wash";

struct Input
{
    double throughput_kg_h         = 20000.0;     // 20 t/h typical plant
    double final_fineness_modulus  = 2.7;         // concrete sand target
    double water_l_kg              = 1.2;         // 1.2 L water per kg sand
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sand_wash
}  // namespace koocadcam::skill
