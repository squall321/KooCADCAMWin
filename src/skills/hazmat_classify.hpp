#pragma once
// @lat: [[engine/skills#hazmat_classify]]
//
// hazmat_classify — hazardous-material classification skill.  Records the
// UN number, hazard class, and packing group of a shipment so transport
// compliance (IATA-DGR, IMDG, ADR, 49 CFR) downstream can verify the
// dangerous-goods declaration matches what was actually loaded.
//
// This is a METADATA-ONLY skill — no geometric change.
//
//      ┌─────────────────────────────┐
//      │  hazardous product ready    │
//      └───────────────┬─────────────┘
//                      │
//                      ▼
//      ┌─────────────────────────────┐    un_number:
//      │  hazmat classification      │      4-digit UN ID (e.g. UN1090)
//      │  un_no + class + pkg group  │    hazard_class:
//      │                             │      "1" .. "9" (DOT classes)
//      └─────────────────────────────┘    packing_group:
//                                           "I" | "II" | "III"
//
// Input:
//   un_number     — std::string, 4-digit UN identifier (e.g. "UN1090")
//   hazard_class  — std::string, DOT class "1" through "9"
//   packing_group — std::string, "I" (high) | "II" (med) | "III" (low) | "" (n/a)
//
// DFM checks (error-level):
//   DFM-INPUT             un_number empty
//   DFM-HM-UNNUMBER       un_number malformed (not "UN" + 4 digits)
//   DFM-INPUT             hazard_class empty
//   DFM-HM-CLASS          hazard_class not in {"1"..."9"}
//   DFM-HM-PKG            packing_group not in {"I","II","III",""}
//
// DFM checks (info-level):
//   DFM-HM-PKG-MISMATCH   class 1/2/7 with non-empty packing_group
//                         (these classes do not assign a packing group)
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hazmat_classify {

constexpr const char* kSkillId = "hazmat_classify";

struct Input
{
    std::string  un_number     = "";       // "UN1090"
    std::string  hazard_class  = "";       // "1" .. "9"
    std::string  packing_group = "";       // "I" | "II" | "III" | ""
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hazmat_classify
}  // namespace koocadcam::skill
