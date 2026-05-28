#pragma once
// @lat: [[engine/skills#tube_bend]]
//
// tube_bend — bend a straight tube into an arc around a bend axis at a
// given axial position.  METADATA-ONLY: the slice-1 geometric model does NOT
// rotate the OCCT shape (rebuilding a swept torus union split-tube is
// expensive and fragile in the presence of internal bores).  We record the
// bend parameters in the signature so downstream CAM/process planners can
// emit the correct mandrel/draw-bender setup.
//
//                              ╱        ← rotated tube section
//                             ╱           (centerline traces an arc of
//                            ╱             radius = bend_radius_mm)
//                          ⌒        ← bend zone (centred on axial_position)
//                       ────
//                            ← fixed tube section
//
// Signature pattern:
//   - pattern.kind             = "tube_bend"
//   - pattern.bend_angle_deg   = <input>
//   - pattern.bend_radius_mm   = <input>
//   - pattern.axial_position_mm = <input>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT     : bend_radius_mm > 0
//   DFM-INPUT     : bend_angle_deg ∈ (0, 180]
//   DFM-TUBE-BEND : bend_radius_mm ≥ 1.5 × bbox.outerRadiusXY() (CLR ≥ 1.5×OD/2
//                   — tight-radius rule for thin-wall tube bending; below
//                   this the inside wall buckles)
//   DFM-TUBE-BEND : axial_position_mm inside stock Z range
//
// Recognize:
//   Cannot be recovered from geometry alone (we don't transform the shape).
//   Returns ONLY signatures replayed from `workpiece.features()` filtered by
//   this skill_id — confidence 1.0.

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tube_bend {

constexpr const char* kSkillId = "tube_bend";

struct Input
{
    double  bend_angle_deg     = 90.0;   // angle in degrees (0, 180]
    double  bend_radius_mm     = 0.0;    // centerline bend radius
    double  axial_position_mm  = 0.0;    // Z-coordinate of bend center
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tube_bend
}  // namespace koocadcam::skill
