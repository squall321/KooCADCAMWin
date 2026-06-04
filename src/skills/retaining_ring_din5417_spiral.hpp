#pragma once
// @lat: [[engine/skills#retaining_ring_din5417_spiral]]
//
// retaining_ring_din5417_spiral — REAL compound: multi-turn spiral retaining
// ring groove per ISO 5417 / DIN 5417.  The groove is wider than a single-
// turn ring because it must accommodate (turn_count × ring_thickness) of
// stacked spiral winds.  Cut depth is sized so the groove leaves enough
// shaft wall + ring engagement (typically ring_thickness above the groove
// bottom).

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace retaining_ring_din5417_spiral {

constexpr const char* kSkillId      = "retaining_ring_din5417_spiral";
constexpr const char* kRingStandard = "DIN 5417 / ISO 5417";

struct Input
{
    FaceDatum   face_id;                          // outer cylindrical face
    double      shaft_z_position_mm = 0.0;        // groove centre Z
    double      shaft_dia_mm        = 0.0;        // nominal shaft Ø
    double      ring_thickness_mm   = 0.0;        // per-turn axial thickness
    int         turn_count          = 2;          // 2 or 3
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace retaining_ring_din5417_spiral
}  // namespace koocadcam::skill
