#pragma once
// @lat: [[engine/skills#splice_telecom]]
//
// splice_telecom — joining (splicing) telecom cables: fusion / mechanical
// fibre splices, coax F-connector splices, or copper crimp / IDC splices.
// Captures the number of splices, the expected insertion loss per splice,
// and the splice method used.  The workpiece B-rep is unchanged — only
// the splice-link record is stamped.
//
//          ───────────╳───────────╳───────────  cable A ── splice ── cable B
//                     │           │
//                     │           │
//                  splice 1    splice 2   (loss_db_per_splice each)
//
// Signature pattern:
//   - pattern.kind                 = "splice_telecom"
//   - pattern.splice_count
//   - pattern.loss_db_per_splice
//   - pattern.total_loss_db        = count × loss_db_per_splice
//   - pattern.splice_method        = "fusion" | "mechanical" | "crimp" | ...
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   splice_count > 0                       (DFM-INPUT error)
//   splice_count ≤ 50                      (DFM-SPLICE-COUNT warning if > 50)
//   loss_db_per_splice ≥ 0                 (DFM-INPUT error if negative)
//   loss_db_per_splice > 0.5               (DFM-SPLICE-LOSS warning — fusion target is < 0.1 dB)
//   splice_method non-empty                (DFM-INPUT error)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
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

namespace splice_telecom {

constexpr const char* kSkillId = "splice_telecom";

struct Input
{
    int          splice_count        = 1;
    double       loss_db_per_splice  = 0.05;
    std::string  splice_method       = "fusion";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace splice_telecom
}  // namespace koocadcam::skill
