#pragma once
// @lat: [[engine/skills#rough_stock_cut]]
//
// rough_stock_cut — metadata record of an upstream saw / shear operation
// that took a small piece (the current workpiece) out of a larger raw
// stock (a 6 m bar, a 1.2 m × 2.4 m plate, …).  This skill does NOT modify
// the workpiece geometry; it simply records the parent stock dimensions,
// the cut dimensions, and the resulting waste percentage so that downstream
// CAPP can roll up material-usage statistics.
//
//        ┌─────────────────────────────────┐
//        │                                 │
//        │   raw stock (e.g. 6000 mm bar)  │
//        │                                 │
//        └───┬───────────────────────────┬─┘
//            │                           │
//            ▼                           ▼
//          saw cut                     saw cut
//            │                           │
//            ▼     ┌──────────────┐
//                  │  workpiece   │ ← this skill's workpiece
//                  └──────────────┘
//          (record original_stock_size, cut_dimension, waste_pct)
//
// Signature pattern:
//   - pattern.kind                 = "rough_stock_cut"
//   - pattern.original_stock_size  = [dx, dy, dz]   (mm — full raw stock)
//   - pattern.cut_dimension        = [dx, dy, dz]   (mm — cut piece, this wp)
//   - pattern.waste_pct            = (orig_vol − cut_vol) / orig_vol × 100
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   DFM-INPUT      any dim ≤ 0                       (error)
//   DFM-INPUT      cut dim > original dim            (error)
//   RC-WASTE-HIGH  waste_pct > 50                    (warning — high yield loss)
//
// Recognize: metadata replay only.

#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rough_stock_cut {

constexpr const char* kSkillId = "rough_stock_cut";

struct Input
{
    std::array<double, 3>  original_stock_size = { 0.0, 0.0, 0.0 };
    std::array<double, 3>  cut_dimension       = { 0.0, 0.0, 0.0 };
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rough_stock_cut
}  // namespace koocadcam::skill
