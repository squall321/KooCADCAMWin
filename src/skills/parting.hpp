#pragma once
// @lat: [[engine/skills#parting]]
//
// parting — separate the workpiece into two parts by cutting through it with
// a parting blade.
//
//        ╓───────────────╖              ╓───────────────╖   ← upper sub-solid
//        ║               ║              ║               ║
//        ║               ║              ╚═══════════════╝   ← cut_z + width/2
//        ║               ║      →
//        ║               ║              ╓───────────────╖   ← cut_z − width/2
//        ║               ║              ║               ║
//        ╚═══════════════╝              ╚═══════════════╝   ← lower sub-solid
//
// Geometrically: full-diameter annular cut (inner R = 0 — i.e. a disc) over
// the Z range [cut_z − width/2, cut_z + width/2].  This produces 2 separated
// solids; OCCT returns them in a TopoDS_Compound.
//
// Signature pattern:
//   - two coaxial planar faces (the cut surfaces) at z = cut_z ± width/2,
//     normals -Z (upper part bottom) and +Z (lower part top).
//   - The resulting workpiece is a TopoDS_Compound with ≥ 2 solid sub-shapes
//     coaxial along Z.
//
// DFM:
//   DFM-INPUT  : cut_width_mm ≥ 1.0 (parting blade min)
//   DFM-INPUT  : cut_z within stock bbox Z range

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace parting {

constexpr const char* kSkillId = "parting";

struct Input
{
    double  cut_z_mm     = 0.0;    // Z of the parting kerf center
    double  cut_width_mm = 1.5;    // blade kerf width
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace parting
}  // namespace koocadcam::skill
