#pragma once
// @lat: [[engine/skills#flow_form]]
//
// flow_form — flow forming (also "tube spinning" or "shear spinning").
// A cylindrical preform is rotated against a mandrel while rollers
// progressively compress its wall, simultaneously REDUCING wall thickness
// and ELONGATING the workpiece along the axis.  Volume is preserved:
//
//   wall_area · length = constant   →   length_new = length_old × t_old / t_new
//
//                 ┌────────────┐
//                 │  preform   │ ──────► flow form ──────► longer + thinner
//                 │ t_orig L   │                            t_final L_new
//                 └────────────┘
//                  ↑ rollers
//
// Synthesis (slice-1, geometric):
//   Rebuild the workpiece as a NEW hollow cylinder with the same OUTER
//   diameter but reduced wall thickness and proportionally increased
//   length.  Bypasses OCCT's lack of strict non-uniform scaling.
//
// Signature pattern:
//   - kind = flow_form, axis = Z
//   - records original_thickness_mm + final_thickness_mm
//   - records original_length_mm + final_length_mm
//   - elongation_pct = (L_new − L_old) / L_old × 100
//
// DFM checks:
//   DFM-INPUT       : final_thickness_mm > 0
//   DFM-FF-MIN      : final_thickness_mm ≥ 0.2 × original_thickness_mm
//                     (max ~80% reduction per pass; greater needs anneal)
//   DFM-FF-TUBE     : workpiece must look like a tube (have an OD and an
//                     ID — i.e. inner radius > 0).  Solid bar cannot flow-
//                     form without first piercing.
//
// Recognize:
//   (a) Metadata replay.
//   (b) Geometric fallback: detect outer + inner Z-axis cylindrical faces
//       with the bbox length significantly greater than the OD — emit a
//       low-confidence candidate.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace flow_form {

constexpr const char* kSkillId = "flow_form";

struct Input
{
    double  outer_dia_mm        = 0.0;   // preform outer diameter (informational)
    double  original_thickness_mm = 0.0; // current wall thickness
    double  final_thickness_mm  = 0.0;   // target wall thickness (< original)
    double  original_length_mm  = 0.0;   // current length along Z
    int     pass_count          = 1;     // number of forming passes (info)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace flow_form
}  // namespace koocadcam::skill
