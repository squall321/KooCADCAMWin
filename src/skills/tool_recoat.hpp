#pragma once
// @lat: [[engine/skills#tool_recoat]]
//
// tool_recoat — strip the existing coating from a used cutting tool and
// re-deposit a fresh wear-resistant coating (TiN, TiAlN, AlCrN, DLC, …).
// Combined refurbishment step common to indexable inserts and HSS / carbide
// tooling.  Geometry on the bench is unchanged; the achieved coating is
// recorded in the signature for QA and CAM tool-library updates.
//
//   ┌────────────┐    strip    ┌────────────┐  recoat   ┌────────────┐
//   │ worn tool  │  ────────▶  │ bare blank │  ──────▶  │ refurbished│
//   │ (TiN worn) │  chemical/  │            │   PVD/    │ (TiAlN)    │
//   └────────────┘  electro    └────────────┘   CVD     └────────────┘
//
// Signature pattern:
//   - pattern.kind             = "tool_recoat"
//   - pattern.tool_id          = <input>
//   - pattern.old_coating      = <input>
//   - pattern.new_coating      = <input>
//   - pattern.stripping_method = "chemical" | "electrochemical" | "blast" | "thermal"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   tool_id non-empty                       (DFM-INPUT error if not)
//   new_coating non-empty                   (DFM-INPUT error if not)
//   stripping_method in known set           (DFM-RC-STRIP info if unknown)
//   new_coating in known coating set        (DFM-RC-COATING info if unknown)
//   old_coating same as new_coating         (DFM-RC-SAMECOAT info — wasted strip?)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tool_recoat {

constexpr const char* kSkillId = "tool_recoat";

struct Input
{
    std::string tool_id           = "EM-6MM-4F-001";
    std::string old_coating       = "TiN";
    std::string new_coating       = "TiAlN";
    std::string stripping_method  = "chemical";   // chemical|electrochemical|blast|thermal
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tool_recoat
}  // namespace koocadcam::skill
