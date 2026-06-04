#pragma once
// @lat: [[engine/skills#ext_retaining_ring_groove_din471]]
//
// ext_retaining_ring_groove_din471 — REAL compound: external (shaft)
// retaining-ring groove cut per DIN 471.  Embeds the central
// _retaining_rings.hpp table; choosing `ring_size_key` ("6mm" .. "100mm")
// drives groove_dia / groove_width / groove_depth automatically.
//
// Apply: cut an annular groove on the outer cylindrical surface, axis along
// the part Z axis, centered at shaft_z_position_mm.
//
// Signature pattern (recognize-compatible):
//   kind            = "ext_retaining_ring_groove_din471"
//   is_compound     = true
//   ring_standard   = "DIN 471"
//   ring_size_key   = e.g. "16mm"
//   derived_dims    = { shaft_dia, groove_dia, width, depth, thk }
//   ring_volume_mm3 = analytical removed volume

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ext_retaining_ring_groove_din471 {

constexpr const char* kSkillId      = "ext_retaining_ring_groove_din471";
constexpr const char* kRingStandard = "DIN 471";

struct Input
{
    FaceDatum   face_id;                         // cylindrical outer face
    double      shaft_z_position_mm = 0.0;       // groove centre Z
    std::string ring_size_key;                   // "6mm" .. "100mm"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ext_retaining_ring_groove_din471
}  // namespace koocadcam::skill
