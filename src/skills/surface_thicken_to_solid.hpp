#pragma once
// @lat: [[engine/skills#surface_thicken_to_solid]]
//
// surface_thicken_to_solid — Offset a face by a positive thickness and fuse
// the resulting prism onto the workpiece, producing a thin solid skin.
// This is the SolidWorks "Thicken" surface op.
//
//                source face
//              ╔═════════════╗
//              ║             ║   <- direction = "outward"
//              ╚═════════════╝
//              ───── thickness_mm
//              ╔═════════════╗
//              ║   prism     ║
//              ╚═════════════╝
//
// Algorithm:
//   1. Resolve `source_face` to a TopoDS_Face on the workpiece.
//   2. Compute the outward face normal at its center.
//   3. Build a prism by sweeping that face along ±normal by `thickness_mm`.
//   4. Fuse the prism with the workpiece (fall back to compound on failure).
//
// Signature pattern:
//   pattern.kind             = "surface_thicken_to_solid"
//   pattern.is_compound      = true
//   pattern.source_face_count= 1
//   pattern.thickness_mm     = (signed by direction)
//   pattern.derived_volume   = volume of the thickened prism
//
// DFM:
//   DFM-INPUT       : source face datum must resolve.
//   DFM-THICKNESS   : thickness_mm must be > 0.
//   DFM-DIRECTION   : direction must be "outward" or "inward".

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace surface_thicken_to_solid {

constexpr const char* kSkillId = "surface_thicken_to_solid";

struct Input
{
    FaceDatum    source_face;
    double       thickness_mm = 1.0;
    std::string  direction    = "outward";   // "outward" | "inward"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace surface_thicken_to_solid
}  // namespace koocadcam::skill
