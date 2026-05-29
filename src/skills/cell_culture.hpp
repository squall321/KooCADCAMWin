#pragma once
// @lat: [[engine/skills#cell_culture]]
//
// cell_culture — record an in-vitro cell culture run.  The workpiece (a
// culture flask / well plate / bioreactor body) is geometrically unchanged;
// the signature records cell line, media composition, time in culture and
// the target confluence at harvest.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │  seeded     │     cell_culture      │  confluent  │
//        │   flask     │ ─────────────────────▶│   monolayer │
//        │             │  (line, media,        │             │
//        │             │   days, %conf)        │             │
//        └─────────────┘                       └─────────────┘
//          shape identical ; culture metadata stamped
//
// Signature pattern:
//   pattern.kind                   = "cell_culture"
//   pattern.cell_line              = string
//   pattern.media_type             = string  ("DMEM" | "RPMI-1640" | ...)
//   pattern.days_in_culture        = double
//   pattern.target_confluence_pct  = double
//   pattern.geometry_changed       = false
//
// DFM:
//   DFM-INPUT          cell_line / media_type empty (error)
//   DFM-CULT-TIME      days_in_culture <= 0 (error)
//   DFM-CULT-TIME      days_in_culture > 60 (info — primary lines senesce)
//   DFM-CULT-CONFL     target_confluence_pct outside (0, 100] (error)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cell_culture {

constexpr const char* kSkillId = "cell_culture";

struct Input
{
    std::string  cell_line              = "HEK293";
    std::string  media_type             = "DMEM";
    double       days_in_culture        = 3.0;
    double       target_confluence_pct  = 80.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cell_culture
}  // namespace koocadcam::skill
