#pragma once
// @lat: [[engine/skills#stone_cut]]
//
// stone_cut — straight-cut record on a dimension-stone slab (granite,
// marble, limestone, sandstone, basalt) using a circular diamond blade
// or wire saw.  Records the cut event (length, blade) as process
// metadata; the CAD-level workpiece B-rep is treated as already
// representing the cut slab and is not re-cut by this skill.
//
// Signature pattern:
//   - pattern.kind             = "stone_cut"
//   - pattern.stone_type       = "granite" | "marble" | "limestone" | "sandstone" | "basalt"
//   - pattern.cut_length_m     = length of cut along the slab
//   - pattern.blade_type       = "diamond_circular" | "diamond_wire" | "abrasive_disc"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT             cut_length_m ≤ 0 (error)
//   DFM-STONE-BLADE       blade_type outside known set (info)
//   DFM-STONE-BLADE-FIT   abrasive_disc on granite — bad lifetime (warning)
//   DFM-STONE-LENGTH      cut_length_m > 6 m (info — needs gantry bridge saw)
//
// Synthesis:
//   NO geometric change.  Clone and stamp.
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

namespace stone_cut {

constexpr const char* kSkillId = "stone_cut";

struct Input
{
    std::string stone_type   = "granite";
    double      cut_length_m = 2.0;
    std::string blade_type   = "diamond_circular";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace stone_cut
}  // namespace koocadcam::skill
