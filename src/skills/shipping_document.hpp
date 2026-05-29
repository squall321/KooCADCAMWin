#pragma once
// @lat: [[engine/skills#shipping_document]]
//
// shipping_document — generate a shipping document set (BOL, CMR, AWB,
// commercial invoice).  Records the document type, recipient (consignee),
// and gross weight so the carrier handoff and downstream POD tracking
// have a stable signature to verify.
//
// This is a METADATA-ONLY skill — no geometric change.
//
//      ┌─────────────────────────────┐
//      │  freight loaded             │
//      └───────────────┬─────────────┘
//                      │
//                      ▼
//      ┌─────────────────────────────┐    doc_type:
//      │  shipping document issued   │      "BOL"  — Bill of Lading (road / rail)
//      │  doc_type + recipient + wt  │      "CMR"  — int'l road consignment
//      │                             │      "AWB"  — Air Waybill
//      └─────────────────────────────┘      "SWB"  — Sea Waybill
//                                           "INV"  — commercial invoice
//
// Input:
//   doc_type    — std::string, "BOL" | "CMR" | "AWB" | "SWB" | "INV"
//   recipient   — std::string, consignee name (non-empty)
//   weight_kg   — double, gross weight on the document (> 0)
//
// DFM checks (error-level):
//   DFM-INPUT          doc_type empty
//   DFM-SD-DOCTYPE     doc_type not in known set
//   DFM-INPUT          recipient empty
//   DFM-INPUT          weight_kg <= 0
//
// DFM checks (info-level):
//   DFM-SD-WEIGHT-HIGH weight_kg > 30000 (heavy lift — special handling)
//   DFM-SD-WEIGHT-AIR  doc_type == "AWB" && weight_kg > 7000 (max ULD)
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

namespace shipping_document {

constexpr const char* kSkillId = "shipping_document";

struct Input
{
    std::string  doc_type   = "BOL";    // "BOL" | "CMR" | "AWB" | "SWB" | "INV"
    std::string  recipient  = "";
    double       weight_kg  = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shipping_document
}  // namespace koocadcam::skill
