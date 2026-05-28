#pragma once
// @lat: [[engine/skills#drill_lathe]]
//
// drill_lathe — center drilling along the rotation axis of a turned part.
//
// A tool-post drill is held in the tailstock and fed straight along the +Z
// rotation axis from the RIGHT (high-Z) end of the workpiece toward the LEFT
// (low-Z) end.  The drill removes a cylinder of `diameter_mm` over a `depth_mm`
// distance from the right face.
//
//        ┌─────────────────┐ z = zMax
//        │ ░░░░░░░░░░░░░░░ │ ← drill bit enters here
//        │ ▓▓▓▓▓ drilled ▓ │
//        │ ▓▓▓▓▓ blind   ▓ │
//        │  hole through ▓ │
//        │ at axis (0,0) ▓ │
//        │                 │
//        └─────────────────┘
//
// Synthesis: build a cylindrical cutter along +Z centred on (0,0), starting
// from z = (zMax - depth) and extending to (zMax + overhang), then cut.
//
// Signature pattern (geometric):
//   - 1 cylindrical face at radius = diameter/2, axis = Z, concentric to (0,0)
//   - 1 circular edge at the entry face (z = zMax)
//   - 1 planar bottom face if not through-hole; ring entry if through.
//
// DFM checks:
//   DFM-INPUT  : diameter_mm > 0.4, depth_mm > 0
//   DFM-LATHE  : depth_mm ≤ stock length (cannot drill past the part)
//   DFM-LATHE  : drill diameter < stock outer (no headstock collision)

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace drill_lathe {

constexpr const char* kSkillId = "drill_lathe";

struct Input
{
    double  diameter_mm = 0.0;
    double  depth_mm    = 0.0;     // along -Z from the right face
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drill_lathe
}  // namespace koocadcam::skill
