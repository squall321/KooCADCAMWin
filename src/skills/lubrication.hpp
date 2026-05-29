#pragma once
// @lat: [[engine/skills#lubrication]]
//
// lubrication — apply a lubricant to a mating surface, bearing, or thread.
// Records lubricant type, volume, and application method so a maintenance
// log can be reconstructed and over/under-lubrication can be detected at
// the next service interval.
//
//        ┌──────────────┐                          ┌──────────────┐
//        │ dry surface  │   lubrication            │ same surface │
//        │              │   ────────────────▶      │ ░ oil film ░ │
//        │              │   (oil/grease/dry-film,  │              │
//        └──────────────┘    volume, method)       └──────────────┘
//          shape identical;  lubricant metadata stamped
//
// Lubricant types (slice-1):
//   "oil"      — mineral / synthetic oil, drip or oil-bath
//   "grease"   — NLGI grade 0–3, packed into bearings or applied with brush
//   "dry-film" — molybdenum disulfide / PTFE, sprayed and air-dried
//
// Application methods (slice-1):
//   "brush"   — manually painted on
//   "spray"   — aerosol or HVLP
//   "drip"    — feed system / oil can
//   "inject"  — grease gun / pressurised injector
//
// Signature pattern:
//   - pattern.kind                = "lubrication"
//   - pattern.lube_type           = "oil" | "grease" | "dry-film"
//   - pattern.volume_ml           = <double>
//   - pattern.application_method  = "brush" | "spray" | "drip" | "inject"
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   DFM-INPUT      volume_ml ≤ 0                  (error)
//   DFM-LUBE-TYPE  lube_type not in known set     (info)
//   DFM-LUBE-METH  method not in known set        (info)
//   DFM-LUBE-VOL   volume_ml > 1000               (info — > 1 L on a single
//                  application is unusual outside bulk reservoirs)
//
// Synthesis:
//   NO geometric change.  Clones the workpiece and stamps the signature.
//
// Recognize:
//   Metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lubrication {

constexpr const char* kSkillId = "lubrication";

struct Input
{
    std::string  lube_type           = "oil";        // "oil"|"grease"|"dry-film"
    double       volume_ml           = 5.0;
    std::string  application_method  = "brush";      // "brush"|"spray"|"drip"|"inject"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lubrication
}  // namespace koocadcam::skill
