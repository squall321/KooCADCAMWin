#pragma once
// @lat: [[engine/skills#leach_extract]]
//
// leach_extract — hydrometallurgical leach.  Crushed/ground ore is contacted
// with a lixiviant (acid, cyanide, or alkaline solution) for a controlled
// residence time; the target metal dissolves and is later recovered by SX/EW
// or precipitation.  KooCADCAM does not simulate ionic chemistry; this skill
// stamps the leach recipe (pH, time, expected recovery) so the recipe can
// be replayed by downstream solvent-extraction skills (future work).
//
//        ore  →  ◉◉◉ leach tank (pH, leach_time_h) →  pregnant solution
//                                                     + tailings
//
// Signature pattern:
//   pattern.kind             = "leach_extract"
//   pattern.ph               = double          (0–14)
//   pattern.leach_time_h     = double
//   pattern.recovery_pct     = double          (0, 100]
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             ph outside [0, 14], leach_time_h ≤ 0,
//                         recovery_pct outside (0, 100]
//   DFM-PH-EXTREME        ph < 1 or ph > 13 (severe corrosion advisory, info)
//   DFM-LEACH-LONG        leach_time_h > 240 (>10 days, info)
//   DFM-RECOVERY-LOW      recovery_pct < 50 (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace leach_extract {

constexpr const char* kSkillId = "leach_extract";

struct Input
{
    double ph            = 2.0;     // typical acid leach for Cu/Ni
    double leach_time_h  = 24.0;
    double recovery_pct  = 80.0;    // 0–100
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace leach_extract
}  // namespace koocadcam::skill
