#pragma once
// @lat: [[engine/skills#bearing_seat]]
//
// bearing_seat — combined precision BORE + adjacent SHOULDER face for a
// rolling-element bearing.  Semantically equivalent to bore_cylindrical
// followed by face_turn against the bore floor:
//
//                          ┌─ entry_face (planar)
//                          ▼
//                    ──────┬───────────┬──────
//                          │           │
//                          │  bore     │       ↑ bore_depth_mm
//                          │           │       │
//                          │           │       │
//                          ├───────────┤       ↓ shoulder face (perpendicular)
//                          │           │
//                          │   ⌀ ID    │  (the shaft passes through here)
//                          │           │
//
// Metadata records the bearing identification (e.g., "6204"), the precision
// fit class (k5/k6/m6/n6/p6), and an axial preload force (N).  These are
// captured in the FeatureSignature for downstream tolerance-stackup and
// process planning.
//
// Signature pattern:
//   - 1 cylindrical face (bore wall, dia = outer_dia_mm + 2*allowance)
//   - 1 circular edge at the bore entry
//   - 1 circular edge where bore meets shoulder
//   - 1 planar SHOULDER face (perpendicular to bore axis)
//
// DFM checks:
//   DFM-BS-DIA  : outer_dia_mm ≥ 4 mm
//   DFM-BS-FIT  : fit_class in { k5,k6,m6,n6,p6,H7 }

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bearing_seat {

constexpr const char* kSkillId = "bearing_seat";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm     = 0.0;
    double      position_y_mm     = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };
    double      outer_dia_mm      = 0.0;     // bearing outer diameter
    double      bore_depth_mm     = 0.0;     // axial depth of the seat
    std::string bearing_id        = "6204";  // ISO bearing designation
    std::string fit_class         = "k6";    // precision fit
    double      preload_N         = 0.0;     // axial preload force
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bearing_seat
}  // namespace koocadcam::skill
