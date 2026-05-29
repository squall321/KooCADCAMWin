#pragma once
// @lat: [[engine/skills#breather_vent_compound]]
//
// breather_vent_compound — COMPOUND, SPEC-DRIVEN feature
//
// A breather vent is a compact pressure-equalising vent for sealed
// electronics / battery enclosures.  It consists of three concentric
// features that, taken together, host a Gore-tex / PTFE membrane and an
// O-ring gasket:
//
//                ▲   counterbore (host for membrane filter)
//                │  ┌─────────┐
//                │  │ groove  │ ← retention groove for O-ring gasket
//                │  │ ┌─────┐ │
//   wall_top  ───┼──┤ │     │ ├──── outer (groove dia)
//                │  │ │     │ │
//                │  └─│     │─┘
//                │    │     │     ← through hole (air pass-through)
//                │    │     │
//                │  ──┴─────┴──── wall_bottom (vent exits to outside)
//
// Spec library (industry breather-vent sizes — ISO M-thread bezel diameters):
//   "BV-M8"  : through_dia=4 mm,  counterbore_dia=8 mm,  cb_depth=2 mm,
//              groove_dia=10 mm, groove_width=0.8 mm, groove_depth=0.5 mm
//   "BV-M12" : through_dia=6 mm,  counterbore_dia=12 mm, cb_depth=3 mm,
//              groove_dia=14 mm, groove_width=1.0 mm, groove_depth=0.7 mm
//   "BV-M16" : through_dia=8 mm,  counterbore_dia=16 mm, cb_depth=4 mm,
//              groove_dia=18 mm, groove_width=1.5 mm, groove_depth=1.0 mm
//   "BV-M20" : through_dia=10 mm, counterbore_dia=20 mm, cb_depth=5 mm,
//              groove_dia=22 mm, groove_width=2.0 mm, groove_depth=1.2 mm
//
// Compound chain:
//   fuse(through_cyl, counterbore_cyl)   → step-bore cutter
//   cut(workpiece, step_bore)            → drilled + counterbored hole
//   cut(workpiece, annular_groove)       → second cut for the O-ring groove
//
// DFM gates:
//   DFM-BV-SP      : unknown spec key.
//   DFM-BV-WALL    : counterbore_depth + groove_depth > wall thickness
//                    (auto-detect from workpiece bbox along axis).
//   DFM-BV-THROUGH : through_dia < 0.8 mm (DFM-002 derived).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace breather_vent_compound {

constexpr const char* kSkillId = "breather_vent_compound";

struct Input
{
    FaceDatum   entry_face;
    std::string spec_key;            // "BV-M8" | "BV-M12" | "BV-M16" | "BV-M20" | ""
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    // Optional overrides used when spec_key is empty.
    double      through_dia_mm       = 0.0;
    double      counterbore_dia_mm   = 0.0;
    double      counterbore_depth_mm = 0.0;
    double      groove_dia_mm        = 0.0;
    double      groove_width_mm      = 0.0;
    double      groove_depth_mm      = 0.0;
};

struct BreatherSpec {
    double through_dia_mm       = 0.0;
    double counterbore_dia_mm   = 0.0;
    double counterbore_depth_mm = 0.0;
    double groove_dia_mm        = 0.0;
    double groove_width_mm      = 0.0;
    double groove_depth_mm      = 0.0;
};

BreatherSpec breatherSpecFor(const std::string& key);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace breather_vent_compound
}  // namespace koocadcam::skill
