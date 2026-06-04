#pragma once
// @lat: [[engine/skills#nas_bolt_pattern_circular]]
//
// nas_bolt_pattern_circular — COMPOUND aerospace NAS bolt pattern on a pitch
// circle diameter (PCD).  N evenly-spaced clearance holes (NAS6000 series
// shank-clearance) around a circular flange / engine boss.  Used on engine
// accessory drives, gearbox flanges, hydraulic actuator mounts.
//
//   sub-feature i (i = 0 .. N-1):
//     straight through-bore at clearance_dia, positioned at
//     (center + PCD/2 × cos(θᵢ), center + PCD/2 × sin(θᵢ))
//     where θᵢ = (2π/N) × i, via gp_Trsf::SetRotation around the face axis.
//
// Sequential pr::cut(wp, cutter) loop — N separate cuts.  Holes do NOT
// overlap (PCD > bolt_dia × bolt_count / π), so they would be safe in
// cutMany too, but per-skill contract we use sequential cuts for slice-14
// uniformity.
//
// clearance grades (radial diametral expansion over bolt_dia):
//   "close"  : +0.0 mm (interference, dowel-like)
//   "medium" : +0.1 mm (NAS618 normal fit)
//   "free"   : +0.2 mm (assembly tolerance fit)
//
// Signature:
//   pattern.kind                   = "nas_bolt_pattern_circular"
//   pattern.is_compound            = true
//   pattern.aerospace_feature_type = "nas_bolt_circle"
//   pattern.fastener_dia           = bolt_dia_mm
//   pattern.bolt_count             = N
//   pattern.pcd_mm                 = PCD
//   pattern.derived_volume_removed_mm3

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace nas_bolt_pattern_circular {

constexpr const char* kSkillId = "nas_bolt_pattern_circular";

struct Input
{
    int         face_id                   = -1;
    double      center_x_mm               = 0.0;
    double      center_y_mm               = 0.0;
    double      pcd_mm                    = 0.0;
    int         bolt_count                = 6;       // 4 / 6 / 8
    double      bolt_dia_mm               = 6.35;    // typical 1/4" NAS
    std::string bolt_hole_clearance_grade = "medium"; // close|medium|free
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace nas_bolt_pattern_circular
}  // namespace koocadcam::skill
