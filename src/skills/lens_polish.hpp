#pragma once
// @lat: [[engine/skills#lens_polish]]
//
// lens_polish — final-stage polishing of an optical surface to drive
// roughness down into the sub-nanometre / Angstrom regime using pitch
// laps + cerium-oxide or colloidal-silica slurry.  Geometry is unchanged
// (the figure was already established by lens_grind); only the surface
// micro-roughness — and therefore scatter / transmitted-wavefront error —
// is improved.
//
// Signature pattern:
//   - pattern.kind                  = "lens_polish"
//   - pattern.target_face_id        = face that was polished
//   - pattern.target_roughness_nm   = RMS roughness target (Sa / Ra)
//   - pattern.polishing_compound    = "cerium_oxide" | "colloidal_silica" |
//                                     "diamond_slurry" | "alumina"
//   - pattern.geometry_changed      = false
//
// DFM checks:
//   target_roughness_nm > 0                          (error)
//   target_roughness_nm ≤ 50 nm                      (info — high-end polish)
//   target_roughness_nm ≥ 0.2 nm                     (error if below — physically
//                                                     unachievable with pitch lap)
//   polishing_compound in known set                  (info if unknown)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lens_polish {

constexpr const char* kSkillId = "lens_polish";

struct Input
{
    FaceDatum   entry_face;
    double      target_roughness_nm = 2.0;     // RMS surface roughness
    std::string polishing_compound  = "cerium_oxide";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lens_polish
}  // namespace koocadcam::skill
