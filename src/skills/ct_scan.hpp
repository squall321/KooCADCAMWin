#pragma once
// @lat: [[engine/skills#ct_scan]]
//
// ct_scan — INSPECTION skill: industrial COMPUTED TOMOGRAPHY 3-D X-ray
// scan.  The part is rotated through 360 ° while X-ray projections are
// captured; reconstruction yields a 3-D voxel grid of attenuation values
// that can reveal INTERNAL defects (porosity, inclusions, voids,
// cracks) invisible to surface-only metrology.
//
//       X-ray src ──▶ │     │ ──▶ detector
//                     │part │
//                     └─────┘     (rotates 360°)
//
//   voxel grid (v_size mm/voxel) ──▶  defect map (Ø ≥ v_size)
//
// What we record:
//   - voxel_size_mm  — the smallest measurable feature (NDT resolution)
//   - dose_kvp       — X-ray tube voltage (typically 80–450 kVp;
//                      higher = more penetration but more part-heating
//                      risk for polymer composites)
//   - detect_internal_defects — capability bool (true unless dose / voxel
//                      combination is degenerate)
//
// Geometrically a NO-OP.
//
// Input:
//   voxel_size_mm — target voxel size (sane range: 0.005..1.0 mm)
//   dose_kvp      — tube voltage (sane range: 40..600 kVp)
//   defect_size_target_mm — smallest defect we need to detect
//
// DFM checks:
//   voxel_size_mm in (0, 1] mm (error if not)
//   voxel_size_mm > defect_size_target_mm ⇒ error
//                  (Nyquist: can't see features smaller than voxel)
//   dose_kvp in [40, 600] (error outside)
//   dose_kvp > 450 ⇒ warning (high-energy: shielding + worker dose)
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ct_scan {

constexpr const char* kSkillId = "ct_scan";

struct Input
{
    double  voxel_size_mm         = 0.050;   // 50 µm voxel
    double  dose_kvp              = 225.0;   // mid-energy industrial CT
    double  defect_size_target_mm = 0.150;   // detect ≥ 150 µm porosity
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ct_scan
}  // namespace koocadcam::skill
