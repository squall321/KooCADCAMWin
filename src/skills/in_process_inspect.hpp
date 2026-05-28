#pragma once
// @lat: [[engine/skills#in_process_inspect]]
//
// in_process_inspect — mid-process sampling check.  Drawn from a lot
// during the production run (NOT first piece, NOT final release) using a
// statistical sampling plan such as ANSI/ASQ Z1.4 (MIL-STD-105E).  The
// recorded data drives lot disposition (continue, hold, rework).
//
// This is a METADATA-ONLY skill — no geometric change.
//
//      lot of N parts ──▶ pull `sample_size` parts ──▶ inspect
//                                                          │
//          accept ◀── defects ≤ accept_size  ──┐ defect_count
//          reject ◀── defects >  accept_size  ──┘
//
// Input:
//   lot_number     — std::string, identifier for the production lot
//   sample_size    — int, # parts pulled for inspection
//   accept_size    — int, max defects allowed for accept (Ac, sampling plan)
//   defect_count   — int, # parts in sample found defective
//
// DFM checks (error-level):
//   DFM-INPUT             sample_size <= 0
//   DFM-INPUT             accept_size < 0
//   DFM-INPUT             defect_count < 0 || > sample_size
//   DFM-IPI-LOT           lot_number empty
//
// DFM checks (info-level):
//   DFM-IPI-REJECT        defect_count > accept_size
//                         (lot fails the sampling plan, still recorded)
//   DFM-IPI-SMALL         sample_size < 5 (statistical power concern)
//
// Recognize:
//   Metadata-only — replayFromFeatures with skill_id filter.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace in_process_inspect {

constexpr const char* kSkillId = "in_process_inspect";

struct Input
{
    std::string  lot_number    = "";
    int          sample_size   = 0;
    int          accept_size   = 0;
    int          defect_count  = 0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace in_process_inspect
}  // namespace koocadcam::skill
