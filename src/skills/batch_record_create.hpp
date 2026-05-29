#pragma once
// @lat: [[engine/skills#batch_record_create]]
//
// batch_record_create — eBatch (electronic batch record) entry.  Captures
// who ran what lot when, plus how many parameters were logged for the
// step.  No geometric change; the record is appended to the workpiece
// feature history so traceability survives serialization.
//
//                ┌──────────────────────────────────────┐
//                │  eBatch entry                        │
//                ├──────────────────────────────────────┤
//                │  lot_id            : L-2026-04891    │
//                │  operator_id       : op-247          │
//                │  timestamp         : 2026-05-29T...  │
//                │  params_logged     : 42              │
//                └──────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind                     = "batch_record_create"
//   - pattern.lot_id                   = <input>
//   - pattern.operator_id              = <input>
//   - pattern.timestamp                = <input>  (ISO 8601 string)
//   - pattern.parameters_logged_count  = <input>
//   - pattern.geometry_changed         = false
//
// DFM checks:
//   DFM-INPUT       lot_id empty                       → error
//   DFM-INPUT       operator_id empty                  → error
//   DFM-INPUT       timestamp empty                    → error
//   DFM-INPUT       parameters_logged_count < 0        → error
//   DFM-EBR-EMPTY   parameters_logged_count == 0       → info (empty record)
//
// Synthesis:
//   NO geometric change.
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

namespace batch_record_create {

constexpr const char* kSkillId = "batch_record_create";

struct Input
{
    std::string  lot_id                   = "";
    std::string  operator_id              = "";
    std::string  timestamp                = "";   // ISO 8601 e.g. "2026-05-29T08:14:23Z"
    int          parameters_logged_count  = 0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace batch_record_create
}  // namespace koocadcam::skill
