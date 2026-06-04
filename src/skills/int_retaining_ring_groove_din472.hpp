#pragma once
// @lat: [[engine/skills#int_retaining_ring_groove_din472]]
//
// int_retaining_ring_groove_din472 — REAL compound: internal (bore)
// retaining-ring groove cut per DIN 472.  Drives groove_dia / width / depth
// from the central _retaining_rings.hpp table by `ring_size_key`.
//
// Apply: cut an annular groove on the internal cylindrical surface (bore).
// The groove expands the bore radially outward; the annular cutter outer
// radius = bore_R + depth and inner radius = bore_R, axially of `width`,
// centered at bore_z_position_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace int_retaining_ring_groove_din472 {

constexpr const char* kSkillId      = "int_retaining_ring_groove_din472";
constexpr const char* kRingStandard = "DIN 472";

struct Input
{
    FaceDatum   bore_face_id;                    // cylindrical internal face
    double      bore_z_position_mm = 0.0;        // groove centre Z
    std::string ring_size_key;                   // "8mm" .. "100mm"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace int_retaining_ring_groove_din472
}  // namespace koocadcam::skill
