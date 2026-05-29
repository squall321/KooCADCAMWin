#pragma once
// @lat: [[engine/skills#knit_apparel]]
//
// knit_apparel — record-only knitting operation forming a fabric / garment
// panel from yarn.  Knit topology is sub-OCCT (interlocking loops, not
// solids); this skill stamps the knit-program signature so MES and costing
// can downstream verify gauge, yarn cost, and pattern reference.
//
// Signature pattern:
//   - pattern.kind             = "knit_apparel"
//   - pattern.gauge            = <input>   needles per inch
//   - pattern.yarn_color       = <input>
//   - pattern.pattern_id       = <input>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   pattern_id     non-empty             (DFM-INPUT error)
//   gauge          ∈ [3, 36]             (DFM-KNIT-GAUGE warning if outside)
//   gauge          > 0                   (DFM-INPUT error)
//   yarn_color     non-empty             (DFM-INPUT warning)
//
// Synthesis:
//   NO geometric change — clone workpiece and stamp signature.
//
// Recognize:
//   Metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace knit_apparel {

constexpr const char* kSkillId = "knit_apparel";

struct Input
{
    int          gauge       = 12;          // needles per inch (NPI)
    std::string  yarn_color  = "black";
    std::string  pattern_id  = "KNIT-001";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace knit_apparel
}  // namespace koocadcam::skill
