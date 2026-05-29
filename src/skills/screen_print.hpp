#pragma once
// @lat: [[engine/skills#screen_print]]
//
// screen_print — push ink through a tensioned mesh stencil onto a substrate.
// The macroscopic B-rep is unchanged; only the surface decoration metadata
// is stamped onto the workpiece.
//
//        ┌─────────────┐                ┌─────────────┐
//        │  substrate  │  screen_print  │  decorated  │
//        │             │ ─────────────▶ │  substrate  │
//        └─────────────┘ (mesh,ink,imp) └─────────────┘
//        geometry unchanged ; print metadata stamped
//
// Signature pattern:
//   pattern.kind             = "screen_print"
//   pattern.mesh_count       = int (threads per inch — 60..420 typical)
//   pattern.ink_color        = string (e.g. "black", "PMS-186C")
//   pattern.impressions      = int (number of pulls)
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             mesh_count ≤ 0 or impressions ≤ 0 (error)
//   DFM-INPUT             ink_color empty (error)
//   DFM-MESH-COARSE       mesh_count < 60 (info — broad lines / heavy lay-down)
//   DFM-MESH-FINE         mesh_count > 420 (info — high resolution / specialty)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace screen_print {

constexpr const char* kSkillId = "screen_print";

struct Input
{
    int          mesh_count  = 156;        // threads/inch
    std::string  ink_color   = "black";
    int          impressions = 1;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace screen_print
}  // namespace koocadcam::skill
