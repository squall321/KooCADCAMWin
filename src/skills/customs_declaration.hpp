#pragma once
// @lat: [[engine/skills#customs_declaration]]
//
// customs_declaration — customs clearance documentation skill.  Records
// the HS code, country of origin, and declared value for a shipment so
// downstream brokerage, duty calculation, and export-control screening
// have a stable signature to verify.
//
// This is a METADATA-ONLY skill — no geometric change.  Captures the
// trade-compliance dossier attached to a finished lot.
//
//      ┌─────────────────────────────┐
//      │  finished lot ready to ship │
//      └───────────────┬─────────────┘
//                      │
//                      ▼
//      ┌─────────────────────────────┐    hs_code:
//      │  customs declaration record │      6-10 digit HS / HTS
//      │  hs_code + origin + value   │    country_origin:
//      │                             │      ISO 3166-1 alpha-2 / -3
//      └─────────────────────────────┘    value_usd:
//                                           declared customs value (> 0)
//
// Input:
//   hs_code        — std::string, HS / HTS code (6-10 digits)
//   country_origin — std::string, ISO 3166 country code (e.g. "KR", "US")
//   value_usd      — double, declared customs value in USD (> 0)
//
// DFM checks (error-level):
//   DFM-INPUT          hs_code empty
//   DFM-CUS-HSCODE     hs_code wrong length / non-digit
//   DFM-INPUT          country_origin empty
//   DFM-CUS-ORIGIN     country_origin wrong length (not 2 or 3 chars)
//   DFM-INPUT          value_usd <= 0
//
// DFM checks (info-level):
//   DFM-CUS-VALUE-HIGH value_usd > 1,000,000  (formal entry required)
//   DFM-CUS-VALUE-LOW  value_usd < 800        (de minimis — may skip duty)
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

namespace customs_declaration {

constexpr const char* kSkillId = "customs_declaration";

struct Input
{
    std::string  hs_code        = "";
    std::string  country_origin = "";    // ISO 3166 alpha-2 or alpha-3
    double       value_usd      = 0.0;   // declared customs value
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace customs_declaration
}  // namespace koocadcam::skill
