#pragma once
// @lat: [[engine/skills#filter_change]]
//
// filter_change — maintenance record for replacing a filtration element
// (engine oil filter, intake air filter, fuel filter, or hydraulic
// return-line cartridge).  The filter element is a sub-OCCT pleated
// media body; this skill records the swap + the hours-in-service of the
// removed cartridge so reliability engineering can trend filter life.
//
//             ┌─────────────┐
//             │ ╲ ╱ ╲ ╱ ╲ ╱ │   ← pleated media (sub-OCCT)
//             │ ╱ ╲ ╱ ╲ ╱ ╲ │
//             └──────┬──────┘
//                    │
//                    ↓
//          filter_id "P164375", hours_in_service = 2400
//
// Signature pattern:
//   - pattern.kind              = "filter_change"
//   - pattern.filter_type       = "oil" | "air" | "fuel" | "hydraulic"
//   - pattern.filter_id         = catalog PN
//   - pattern.hours_in_service  = hours on removed cartridge (≥ 0)
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   filter_id  non-empty                      (DFM-INPUT error)
//   filter_type ∈ {oil, air, fuel, hydraulic} (DFM-FILTER-TYPE info if unknown)
//   hours_in_service ≥ 0                      (DFM-INPUT error if < 0)
//   hours_in_service > 8760                   (DFM-FILTER-LIFE info — 1 yr+,
//                                              likely overdue)
//
// Synthesis:
//   NO geometric change.  Clone shape + material and stamp signature.
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

namespace filter_change {

constexpr const char* kSkillId = "filter_change";

struct Input
{
    std::string filter_type       = "oil";   // "oil" | "air" | "fuel" | "hydraulic"
    std::string filter_id         = "P164375";
    double      hours_in_service  = 500.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace filter_change
}  // namespace koocadcam::skill
