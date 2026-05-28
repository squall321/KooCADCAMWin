#pragma once
// @lat: [[engine/skills#cryogenic_machining]]
//
// cryogenic_machining — wrap a sub-skill (drill / mill / turn) with
// cryogenic coolant delivery (liquid nitrogen, LN2, at -196 °C).
//
// Cryogenic coolant replaces flood / mist coolant.  Benefits:
//   - cuts heat-resistant alloys (titanium, Inconel, hardened steel) with
//     2–4× longer tool life vs flood
//   - no residue / clean-up; LN2 evaporates
//   - lower environmental impact than emulsion coolants
//
// This skill is a METADATA WRAPPER.  The sub-skill (the workhorse —
// drill_hole, mill_circular_pocket, turn_external) is named by
// `sub_skill_id`, the input parameters live in `sub_skill_params`.  When
// apply() runs it delegates to the sub-skill, then stamps an OUTER
// signature with skill_id = "cryogenic_machining" + the cryogenic-coolant
// metadata so downstream consumers can attribute the strategy.
//
//        ┌────────────────────────────────────┐
//        │  cryogenic_machining (wrapper)     │
//        │  ┌──────────────────────────────┐  │
//        │  │  drill_hole / mill_pocket /  │  │
//        │  │  turn_external              │  │
//        │  └──────────────────────────────┘  │
//        │  +metadata: LN2 coolant, flow      │
//        │             rate, delivery mode    │
//        └────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "cryogenic_machining"
//   - pattern.sub_skill_id     = e.g. "drill_hole"
//   - pattern.coolant          = "ln2"
//   - pattern.flow_rate_lpm    = caller-supplied
//   - pattern.delivery         = "through_spindle" | "external_jet"
//   - tooling.extra.cryogenic  = true
//
// DFM checks:
//   DFM-INPUT          : sub_skill_id empty → error
//   DFM-CRYO-FLOW      : flow_rate_lpm ∉ (0, 20] → error / warning
//   DFM-CRYO-DELIVERY  : delivery not in known set → info
//
// Synthesis:
//   For phase-1 we implement only the WRAPPER + metadata.  The sub-skill is
//   NOT actually invoked here (that requires dynamic dispatch / a registry
//   we don't yet have); instead we store the sub-skill name + params and
//   leave the workpiece geometry unchanged.  Downstream process planners
//   are expected to expand the wrapper into <sub-skill> + cryogenic tooling
//   directives at codegen time.
//
// Recognize:
//   Metadata-only — replay from `workpiece.features()`.

#include "Skill.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cryogenic_machining {

constexpr const char* kSkillId = "cryogenic_machining";

struct Input
{
    std::string     sub_skill_id     = "";       // "drill_hole" | "mill_circular_pocket" | …
    nlohmann::json  sub_skill_params = nlohmann::json::object();
    double          flow_rate_lpm    = 5.0;      // LN2 flow rate, litres/min
    std::string     delivery         = "through_spindle";  // | "external_jet"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cryogenic_machining
}  // namespace koocadcam::skill
