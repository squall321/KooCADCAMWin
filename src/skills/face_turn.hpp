#pragma once
// @lat: [[engine/skills#face_turn]]
//
// face_turn — facing operation: square the END face of a cylindrical part by
// bringing it to a precise Z plane (lower than its current zMax).
//
//        ╓───────────────╖             ╓───────────────╖   ← current zMax
//        ║               ║             ║░░░░░░░░░░░░░░░║   removed slab
//        ║               ║      →      ╠═══════════════╣   ← face_z (squared)
//        ║               ║             ║               ║
//        ║               ║             ║               ║
//        ║               ║             ║               ║
//        ╚═══════════════╝             ╚═══════════════╝
//
// Synthesis: build a full-diameter cylinder from face_z to (zMax + overhang)
// and cut it from the workpiece.  This squares the top of the part to a flat
// disc at z = face_z.
//
// Signature pattern:
//   - 1 planar disc face at z = face_z, normal +Z
//   - 1 circular edge at the disc perimeter, radius = bbox outer
//
// DFM:
//   DFM-INPUT  : face_z_mm < zMax (otherwise no material to remove)
//   DFM-INPUT  : face_z_mm > zMin (otherwise destroys the part)
//   DFM-INPUT  : surface_finish must be one of recognised codes (info only)

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace face_turn {

constexpr const char* kSkillId = "face_turn";

struct Input
{
    double       face_z_mm       = 0.0;       // target Z of the squared face
    std::string  surface_finish  = "ra_1.6";  // "ra_0.4" / "ra_0.8" / "ra_1.6" / "ra_3.2"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace face_turn
}  // namespace koocadcam::skill
