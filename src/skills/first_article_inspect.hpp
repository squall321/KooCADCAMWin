#pragma once
// @lat: [[engine/skills#first_article_inspect]]
//
// first_article_inspect — first-article inspection (FAI) skill.  Records
// the verification of the FIRST piece coming off a tooling/setup change,
// against the master drawing.  Approval of the first article gates serial
// production of the lot.
//
// This is a METADATA-ONLY skill: no geometric change, no material change.
// The signature carries the count of features checked, how many dimensions
// passed vs. the total, and the approval signature (inspector ID / stamp).
//
//      ┌───────────────────────────┐
//      │  fresh setup / new tool   │
//      │   produces FIRST piece    │
//      └─────────────┬─────────────┘
//                    │
//                    ▼
//      ┌───────────────────────────┐
//      │ FAI: verify every called  │
//      │ dimension against drawing │
//      │  → pass / fail / approve  │
//      └───────────────────────────┘
//
// Input:
//   features_checked      — int, # of distinct features measured (e.g. 12 holes)
//   dimensions_pass       — int, # of dimensions within tolerance
//   dimensions_total      — int, # of dimensions called out on the drawing
//   approval_signature    — std::string, inspector ID / digital signature
//   inspector_name        — std::string, free-form inspector name (optional)
//
// DFM checks (error-level — apply() throws on bad input):
//   DFM-INPUT             dimensions_total <= 0
//   DFM-INPUT             dimensions_pass < 0 || > dimensions_total
//   DFM-INPUT             features_checked < 0
//   DFM-FAI-APPROVAL      approval_signature empty
//
// DFM checks (info-level — operation proceeds):
//   DFM-FAI-INCOMPLETE    dimensions_pass < dimensions_total
//                         (FAI is recorded as a non-passing first article)
//   DFM-FAI-COVERAGE      features_checked == 0 with dimensions_total > 0
//
// Recognize:
//   Metadata-only — recognize() returns matches replayed from
//   workpiece.features() filtered by skill_id, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace first_article_inspect {

constexpr const char* kSkillId = "first_article_inspect";

struct Input
{
    int          features_checked    = 0;
    int          dimensions_pass     = 0;
    int          dimensions_total    = 0;
    std::string  approval_signature  = "";
    std::string  inspector_name      = "";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace first_article_inspect
}  // namespace koocadcam::skill
