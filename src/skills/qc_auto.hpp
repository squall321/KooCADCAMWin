#pragma once
// @lat: [[engine/skills#qc_auto]]
//
// qc_auto — end-of-line vehicle inspection / quality control station.
// Records the VIN (Vehicle Identification Number), total inspection
// items performed, and the number of items that passed.  Macroscopic
// B-rep is unchanged; this is a measurement / record-only station.
//
//          ┌────────────────────────────────────┐
//          │  vehicle               VIN: <id>   │
//          │  ──────────▶  QC ──▶   pass/total  │
//          │                                    │
//          └────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "qc_auto"
//   - pattern.vin              = string (17-char VIN normally)
//   - pattern.inspection_count = int
//   - pattern.pass_count       = int
//   - pattern.pass_rate        = pass_count / inspection_count
//   - pattern.geometry_changed = false
//
// DFM checks:
//   vin non-empty                                      (DFM-INPUT error)
//   vin length != 17                                   (DFM-QC-VIN info)
//   inspection_count > 0                               (DFM-INPUT error if <= 0)
//   pass_count ≥ 0                                     (DFM-INPUT error if < 0)
//   pass_count ≤ inspection_count                      (DFM-QC-COUNT error)
//   pass_count < inspection_count                      (DFM-QC-FAILS info)
//
// Synthesis:
//   NO geometric change.  Clone shape + material, stamp signature.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace qc_auto {

constexpr const char* kSkillId = "qc_auto";

struct Input
{
    std::string  vin              = "KMHXXXXXXXXXXXXXX";  // 17-char VIN
    int          inspection_count = 120;
    int          pass_count       = 120;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace qc_auto
}  // namespace koocadcam::skill
