#pragma once
// @lat: [[engine/skills#circular_remanufacture]]
//
// circular_remanufacture — returns-core refurbishment record for a
// reusable product (e.g. motor, gearbox, battery pack, watch movement)
// that has been cleaned, inspected, replaced-part lists applied, and
// re-certified with a renewed warranty.  The core retains the original
// product identity (core_id) but is now functionally equivalent to new.
// Pure metadata operation: workpiece B-rep is unchanged.
//
//   ┌────────────────┐  circular_remanufacture  ┌────────────────┐
//   │ returned core  │ ───────────────────────▶ │ stamp record   │
//   │ core_id, steps │ (refurb steps, warranty) │ new warranty   │
//   └────────────────┘                          └────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "circular_remanufacture"
//   - pattern.core_id          = string
//   - pattern.refurbish_steps  = list of strings (clean, inspect, replace, …)
//   - pattern.warranty_months
//   - pattern.step_count       = derived
//   - pattern.geometry_changed = false
//
// DFM checks:
//   core_id            non-empty                  (DFM-INPUT error)
//   refurbish_steps    non-empty                  (DFM-INPUT error)
//   warranty_months    > 0                        (DFM-INPUT error)
//   warranty_months    ≤ 60                       (DFM-REMAN-WARRANTY info if exceeded)
//   step_count         ≥ 3 industry minimum       (DFM-REMAN-STEPS info if fewer)
//
// Synthesis: NO geometric change.  Clone + stamp signature.
// Recognize: Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace circular_remanufacture {

constexpr const char* kSkillId = "circular_remanufacture";

struct Input
{
    std::string               core_id          = "CORE-0001";
    std::vector<std::string>  refurbish_steps  = { "clean", "inspect", "replace_seals", "recertify" };
    int                       warranty_months  = 24;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace circular_remanufacture
}  // namespace koocadcam::skill
