#pragma once
// @lat: [[engine/skills#e_clip_groove_din6799]]
//
// e_clip_groove_din6799 — REAL compound: shallow axial groove sized per
// DIN 6799 to accept an E-clip (Sicherungsscheibe für Wellen).  Cuts a
// narrow annular groove on the outer cylindrical surface.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace e_clip_groove_din6799 {

constexpr const char* kSkillId      = "e_clip_groove_din6799";
constexpr const char* kRingStandard = "DIN 6799";

struct Input
{
    FaceDatum   face_id;                         // outer cylindrical face
    double      shaft_z_position_mm = 0.0;       // groove centre Z
    std::string ring_size_key;                   // "1.2mm" .. "9mm"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace e_clip_groove_din6799
}  // namespace koocadcam::skill
