#pragma once
// @lat: [[engine/skills#embroider]]
//
// embroider — record-only decorative stitching of a design onto fabric.
// Embroidery paths are stored as DST/EMB files outside the B-rep; this
// skill stamps the design metadata so MES, costing, and color-change
// scheduling can downstream act.
//
// Signature pattern:
//   - pattern.kind                 = "embroider"
//   - pattern.design_id            = <input>
//   - pattern.stitches_count       = <input>
//   - pattern.thread_colors_count  = <input>
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   design_id            non-empty             (DFM-INPUT error)
//   stitches_count       ≥ 1                   (DFM-INPUT error)
//   stitches_count       ≤ 1_000_000           (DFM-EMB-SIZE warning)
//   thread_colors_count  ≥ 1                   (DFM-INPUT error)
//   thread_colors_count  ≤ 15                  (DFM-EMB-COLORS warning)
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

namespace embroider {

constexpr const char* kSkillId = "embroider";

struct Input
{
    std::string  design_id            = "EMB-001";
    int          stitches_count       = 5000;
    int          thread_colors_count  = 3;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace embroider
}  // namespace koocadcam::skill
