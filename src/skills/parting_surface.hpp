#pragma once
// @lat: [[engine/skills#parting_surface]]
//
// parting_surface — the conceptual plane at which the two halves of an
// injection mould meet.  Geometrically, we materialise it as a large thin
// planar flange that extends past the workpiece silhouette at Z =
// `silhouette_plane_z_mm`, with a cut-out matching the workpiece's
// silhouette at that Z so the flange wraps the part.
//
//   Top view:
//                ┌─────────────────────────────────┐
//                │            extension            │
//                │   ┌─────────────────────────┐   │
//                │   │                         │   │
//                │   │      workpiece          │   │  ← thin flange
//                │   │      silhouette         │   │    (parting surface)
//                │   │      (cut out)          │   │
//                │   └─────────────────────────┘   │
//                │                                 │
//                └─────────────────────────────────┘
//
// Slice-1 simplification: build a thin annular planar disc (a box with a
// rectangular hole matching the workpiece's XY bbox at the parting plane),
// then fuse it onto the workpiece.  The thin flange's thickness is fixed
// to `kFlangeThicknessMm` (0.5 mm).
//
// Signature pattern:
//   pattern.kind                   = "parting_surface"
//   pattern.silhouette_plane_z_mm  = silhouette Z (world)
//
// DFM:
//   DFM-PART-Z         : silhouette_plane_z must lie within workpiece Z bbox.
//   DFM-PART-EXT       : extension ≥ 2 mm (else flange is too thin to handle).

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace parting_surface {

constexpr const char* kSkillId = "parting_surface";

struct Input
{
    double      silhouette_plane_z_mm = 0.0;
    double      extension_mm          = 0.0;     // overhang past workpiece XY bbox in all directions
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace parting_surface
}  // namespace koocadcam::skill
