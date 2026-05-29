#pragma once
// @lat: [[engine/skills#auto_standoff_floating_point]]
//
// auto_standoff_floating_point — CONTEXT-AWARE compound macro that
// guarantees a standoff (raised cylindrical post with a tapped top) at
// `position_xy` even when the workpiece has no Z = 0 baseplate to mount
// it on.
//
// Sub-features (subfeature_count = 4 if base dropped, 3 otherwise):
//   1. [conditional] thin base plate dropped at Z = 0    [fuse]
//   2. cylindrical standoff column (outer)                [fuse]
//   3. cylindrical standoff column (concentric → fused
//      with outer to avoid Boolean splits)
//   4. tapped pilot bore cut from the top                 [cut]
//
// Lookup table — Tapped standoff specs (M2/M2.5/M3/M4/M5/M6, common
// PCB-mount sizes):
//   M2    → pilot 1.6 mm, OD ≥ 4.0 mm, min wall 1.2 mm
//   M2.5  → pilot 2.05, OD ≥ 5.0 mm, min wall 1.5 mm
//   M3    → pilot 2.5,  OD ≥ 6.0 mm, min wall 1.75 mm
//   M4    → pilot 3.3,  OD ≥ 8.0 mm, min wall 2.0 mm
//   M5    → pilot 4.2,  OD ≥ 10.0 mm, min wall 2.5 mm
//   M6    → pilot 5.0,  OD ≥ 12.0 mm, min wall 3.0 mm
//
// DFM:
//   DFM-INPUT       : missing dims.
//   DFM-STANDOFF-SIZE: unknown thread_spec.
//   DFM-STANDOFF-OD : standoff_od < lookup min.
//   DFM-STANDOFF-H  : height ≥ 3 × pilot_dia (else thread engagement bad).
//
// Recognize:
//   - Geometric: a cylinder of OD matching table + concentric smaller
//     cylinder hole + base flat face at Z=0 → standoff.
//   - Fallback: metadata replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace auto_standoff_floating_point {

constexpr const char* kSkillId = "auto_standoff_floating_point";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    double      height_mm       = 10.0;
    gp_Dir      axis_dir        { 0.0, 0.0, 1.0 };  // standoff grows up
    std::string thread_spec     = "M3";
    double      standoff_od_mm  = 6.0;
    double      base_plate_thickness_mm = 1.2;       // dropped if no base at Z=0
};

SkillOutput   apply   (const Workpiece& wp, const Input& in);
DFMReport     validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace auto_standoff_floating_point
}  // namespace koocadcam::skill
