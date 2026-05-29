#pragma once
// @lat: [[engine/skills#freight_loading]]
//
// freight_loading — load freight into a truck or container.  Records
// freight class, total weight, piece count, and the container type
// so dispatch, capacity planning, and weight-balance audits have a
// stable signature to verify.
//
// This is a METADATA-ONLY skill — no geometric change.
//
//      ┌─────────────────────────────┐
//      │  finished cartons / pallets │
//      └───────────────┬─────────────┘
//                      │
//                      ▼
//      ┌─────────────────────────────┐    container_type:
//      │  freight loaded into        │      "20ft" | "40ft" | "40HC"
//      │  truck / container          │      "53ft_truck" | "LTL_pallet"
//      │                             │    freight_class:  NMFC 50–500
//      └─────────────────────────────┘    weight_kg:       gross load
//
// Input:
//   freight_class  — std::string, NMFC class code (e.g. "50", "85", "500")
//   weight_kg      — double, total loaded weight in kg (> 0)
//   count          — int, number of pallets/cartons loaded (> 0)
//   container_type — std::string, "20ft" | "40ft" | "40HC" | "53ft_truck" | "LTL_pallet"
//
// DFM checks (error-level):
//   DFM-INPUT             freight_class empty
//   DFM-INPUT             weight_kg <= 0
//   DFM-INPUT             count <= 0
//   DFM-FL-CONTAINER      container_type unrecognized
//   DFM-FL-OVERLOAD       weight_kg exceeds container payload limit
//
// DFM checks (info-level):
//   DFM-FL-UNDERLOAD      weight_kg < 30% of container payload (poor utilization)
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

namespace freight_loading {

constexpr const char* kSkillId = "freight_loading";

struct Input
{
    std::string  freight_class  = "";       // NMFC class code
    double       weight_kg      = 0.0;
    int          count          = 0;
    std::string  container_type = "40ft";   // "20ft" | "40ft" | "40HC" | "53ft_truck" | "LTL_pallet"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace freight_loading
}  // namespace koocadcam::skill
