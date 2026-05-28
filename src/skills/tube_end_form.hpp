#pragma once
// @lat: [[engine/skills#tube_end_form]]
//
// tube_end_form — special end forming applied to a tube terminus.  Covers
// the family of operations a typical tube-end-form machine can perform on a
// single die set:
//
//   "bell"      a bell-mouth (truncated cone outward expansion)
//   "beaded"    a peripheral rolled bead (annular bulge near the end)
//   "expanded_slot"  a longitudinal slot cut + flared into a small tab/slot
//
// METADATA-ONLY: the three sub-forms have very different topologies; we
// record the chosen form_type plus its dimensional parameters in the
// signature so downstream CAM can pick the correct die and stroke profile.
// (A pure geometric model would require three different rebuild strategies
// — out of scope for the slice-1 skill contract.)
//
// Signature pattern:
//   - pattern.kind        = "tube_end_form"
//   - pattern.form_type   = "bell" | "beaded" | "expanded_slot"
//   - pattern.length_mm
//   - pattern.diameter_mm  // (bell mouth dia; bead OD; slot peripheral dia)
//   - pattern.end          = "z_min" | "z_max"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT          : form_type ∈ {bell, beaded, expanded_slot}
//   DFM-INPUT          : length_mm > 0, diameter_mm > 0
//   DFM-TUBE-EF-LEN    : length_mm ≤ 3 × bbox.outerRadiusXY()
//                        (typical end-form stroke limit)
//   DFM-TUBE-EF-DIA    : for bell/beaded, diameter_mm > 2×outerR (must expand)
//   DFM-TUBE-EF-END    : end ∈ {z_min, z_max}
//
// Recognize:
//   Metadata replay only.

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tube_end_form {

constexpr const char* kSkillId = "tube_end_form";

struct Input
{
    std::string  form_type    = "bell";        // bell | beaded | expanded_slot
    double       length_mm    = 0.0;           // axial length of formed region
    double       diameter_mm  = 0.0;           // characteristic outer dia
    std::string  end          = "z_max";       // z_min | z_max
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tube_end_form
}  // namespace koocadcam::skill
