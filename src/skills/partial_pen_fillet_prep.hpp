#pragma once
// @lat: [[engine/skills#partial_pen_fillet_prep]]
//
// partial_pen_fillet_prep — COMPOUND MACRO that prepares one plate edge
// for a partial-penetration fillet weld per AWS A2.4 (single-bevel fillet
// land).
//
// Geometric sub-features (≥ 2):
//   1. Single bevel — angled prism cut on ONE side of the edge.
//   2. Back radius / root cylinder — small cylindrical relief at the heel
//      so the welder can land the root pass cleanly (DIN EN ISO 9692-1).
//
//                        ┌───────────────────┐
//                        │   \               │  ← single bevel (bevel_angle
//                        │    \              │     measured from the plate
//                        │     \             │     normal)
//                        │      \            │
//                        │       \           │
//                        │  ───   \          │  ↑ land_mm (unmachined heel)
//                        │       (•)         │  ↓ + small back radius
//                        └───────────────────┘
//                                ↕
//                            edge_dir
//
// DFM checks (AWS A2.4 / DIN EN ISO 9692-1):
//   DFM-WELD-ANGLE  : bevel_angle in [25°, 60°].
//   DFM-WELD-LAND   : land_mm in [1.0, plate_t / 2].
//   DFM-WELD-PLATE  : plate_t >= 3 mm.
//
// Recognize: metadata replay only (confidence = 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace partial_pen_fillet_prep {

constexpr const char* kSkillId = "partial_pen_fillet_prep";

struct Input
{
    FaceDatum   entry_face;
    gp_Dir      edge_dir         { 1.0, 0.0, 0.0 };
    double      plate_t          = 0.0;
    double      bevel_angle      = 45.0;   // measured from plate normal
    double      land_mm          = 1.5;
    double      edge_center_x_mm = 0.0;
    double      edge_center_y_mm = 0.0;
    double      edge_length_mm   = 50.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace partial_pen_fillet_prep
}  // namespace koocadcam::skill
