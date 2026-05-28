#pragma once
// @lat: [[engine/skills#batch_release]]
//
// batch_release — final lot disposition skill.  Records the formal
// release of a finished lot to the next process step (or to the
// customer).  Encodes lot_id, lot_size, accept_count, and the
// release_class verdict (A / B / C / scrap).
//
// This is a METADATA-ONLY skill — no geometric change.  Used to close
// the QA chain on a production lot.
//
//      ┌─────────────────────────────┐
//      │  lot finished + inspected   │
//      └───────────────┬─────────────┘
//                      │
//                      ▼
//      ┌─────────────────────────────┐    release_class:
//      │  batch_release verdict      │      "A" — full accept
//      │  yield = accept / lot_size  │      "B" — accept with concessions
//      │                             │      "C" — sort / rework / hold
//      └─────────────────────────────┘      "scrap" — full reject
//
// Input:
//   lot_id          — std::string, traceability ID for this lot
//   lot_size        — int, # parts in the lot, must be > 0
//   accept_count    — int, # parts that passed final inspection
//   release_class   — std::string, "A" | "B" | "C" | "scrap"
//
// DFM checks (error-level):
//   DFM-INPUT             lot_size <= 0
//   DFM-INPUT             accept_count < 0 || > lot_size
//   DFM-BR-LOTID          lot_id empty
//   DFM-BR-CLASS          release_class not in {A, B, C, scrap}
//
// DFM checks (info-level):
//   DFM-BR-YIELD-LOW      yield = accept / lot_size < 0.95
//   DFM-BR-CLASS-MISMATCH yield < 0.95 but release_class == "A"
//                         (suspicious — high yield required for class A)
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

namespace batch_release {

constexpr const char* kSkillId = "batch_release";

struct Input
{
    std::string  lot_id        = "";
    int          lot_size      = 0;
    int          accept_count  = 0;
    std::string  release_class = "A";    // "A" | "B" | "C" | "scrap"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace batch_release
}  // namespace koocadcam::skill
