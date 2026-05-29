#pragma once
// @lat: [[engine/skills#inventory_count]]
//
// inventory_count — physical inventory count skill.  Records expected
// vs actual quantity for a lot and computes the variance percentage so
// cycle-count audits, ERP reconciliation, and shrinkage tracking have
// a stable signature to verify.
//
// This is a METADATA-ONLY skill — no geometric change.
//
//      ┌─────────────────────────────┐
//      │  scheduled cycle count      │
//      └───────────────┬─────────────┘
//                      │
//                      ▼
//      ┌─────────────────────────────┐    variance_pct =
//      │  inventory_count record     │      (actual - expected) / expected × 100
//      │  expected vs actual qty     │
//      │                             │    positive = overage
//      └─────────────────────────────┘    negative = shortage
//
// Input:
//   lot_id        — std::string, traceability ID
//   expected_qty  — int, system quantity per ERP (> 0)
//   actual_qty    — int, physical count on the floor (>= 0)
//
// DFM checks (error-level):
//   DFM-INPUT           lot_id empty
//   DFM-INPUT           expected_qty <= 0
//   DFM-INPUT           actual_qty < 0
//
// DFM checks (info-level):
//   DFM-IC-SHORTAGE     variance_pct < -2.0%  (significant shortage)
//   DFM-IC-OVERAGE      variance_pct > +2.0%  (significant overage)
//   DFM-IC-CRITICAL     |variance_pct| > 10%  (escalate to physical audit)
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

namespace inventory_count {

constexpr const char* kSkillId = "inventory_count";

struct Input
{
    std::string  lot_id       = "";
    int          expected_qty = 0;
    int          actual_qty   = 0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace inventory_count
}  // namespace koocadcam::skill
