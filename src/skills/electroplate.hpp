#pragma once
// @lat: [[engine/skills#electroplate]]
//
// electroplate — electrochemical deposition of one metal onto another.  The
// target face is coated with a thin layer (1 – 200 μm) of plating metal
// (nickel / chrome / zinc / copper / gold).
//
//        ┌─────────────┐
//        │ ════════════│   ← plating metal layer (1 – 200 μm)
//        │             │
//        │  (base      │   ← base metal substrate (unchanged)
//        │   metal)    │
//        └─────────────┘
//
// Signature pattern:
//   - pattern.kind                = "electroplate"
//   - pattern.target_face_id      = face that was plated
//   - pattern.plating_metal       = "nickel" | "chrome" | "zinc" | "copper" | "gold"
//   - pattern.coating_thickness_um
//   - pattern.base_material       = the substrate (free-form string)
//   - pattern.plating_fit         = "good" | "needs_strike" | "unknown"
//   - geometry IDENTICAL to input
//
// DFM checks:
//   coating_thickness_um ∈ [1, 200]               (DFM-EP-THK error if outside)
//   plating_metal in {nickel, chrome, zinc, copper, gold}  (DFM-EP-METAL info if unknown)
//   plating–base compatibility table              (DFM-EP-FIT info)
//
// Synthesis:
//   NO geometric change — clones the workpiece with the signature stamped.
//
// Recognize:
//   Impossible from B-rep.  Returns ONLY metadata-replayed signatures
//   filtered by skill_id, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace electroplate {

constexpr const char* kSkillId = "electroplate";

struct Input
{
    FaceDatum   entry_face;
    std::string plating_metal        = "nickel";   // see header table
    double      coating_thickness_um = 15.0;       // typical bright-nickel
    std::string base_material;                      // optional override of
                                                    // wp.material() for the
                                                    // plating-fit lookup
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace electroplate
}  // namespace koocadcam::skill
