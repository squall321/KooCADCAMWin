#pragma once
// @lat: [[engine/skills#calibration_check]]
//
// calibration_check — gauge/instrument calibration verification skill.
// Records that a measuring instrument (gauge, CMM probe, micrometer,
// etc.) has been verified against a traceable standard, with its last
// calibration date and the date the next calibration is due.
//
// This is a METADATA-ONLY skill — no geometric change.  Drives the
// metrology-system audit trail: every inspection that uses gauge G must
// be preceded by a current (not-yet-due) calibration_check for G.
//
// Input:
//   gauge_id        — std::string, equipment serial / asset tag
//   last_cal_date   — std::string, ISO 8601 date "YYYY-MM-DD"
//   due_date        — std::string, ISO 8601 date "YYYY-MM-DD"
//   traceability    — std::string, traceable standard chain
//                     (e.g. "NIST", "KRISS", "PTB", "DKD")
//   current_date    — std::string, ISO 8601 "YYYY-MM-DD"
//                     (date of THIS check; informally "today")
//
// DFM checks (error-level):
//   DFM-INPUT             gauge_id empty
//   DFM-INPUT             last_cal_date empty
//   DFM-INPUT             due_date empty
//   DFM-CAL-ORDER         due_date strictly before last_cal_date
//
// DFM checks (info-level):
//   DFM-CAL-OVERDUE       current_date >= due_date (calibration overdue)
//   DFM-CAL-TRACE         traceability not in {NIST, KRISS, PTB, DKD, JCSS}
//                         (unrecognized national-lab traceability)
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

namespace calibration_check {

constexpr const char* kSkillId = "calibration_check";

struct Input
{
    std::string  gauge_id      = "";
    std::string  last_cal_date = "";      // ISO 8601 "YYYY-MM-DD"
    std::string  due_date      = "";      // ISO 8601 "YYYY-MM-DD"
    std::string  traceability  = "NIST";  // NIST | KRISS | PTB | DKD | JCSS
    std::string  current_date  = "";      // ISO 8601 "YYYY-MM-DD"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace calibration_check
}  // namespace koocadcam::skill
