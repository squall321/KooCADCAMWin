#pragma once
// @lat: [[engine/skills#label_apply]]
//
// label_apply — adhere a thin LABEL (sticker / nameplate / barcode foil) to
// a planar workpiece face.  Geometrically the label is modelled as a thin
// rectangular slab fused onto the face with a small 1 % sink (spot_weld
// convention) for robust Boolean union.
//
//                  ┌────────────────┐
//                  │     label      │   ↕ label_thickness_mm (≤ 0.5)
//                ──┴────────────────┴── ← entry_face (planar)
//
//                  ←─label_width_mm─→
//
// Signature pattern:
//   - 1 thin rectangular slab on the face
//   - planar top + 4 vertical side faces, all parallel to the entry plane
//   - pattern.kind = "label_apply"
//   - dims (width, height, thickness) + optional text
//
// DFM checks:
//   DFM-LABEL-THICK : label_thickness_mm ≤ 0.5
//   DFM-LABEL-FIT   : label width/height fit within face area
//   DFM-INPUT       : positive dimensions
//
// Recognize:
//   Heuristic + metadata replay.  A thin rectangular protrusion (very low
//   aspect ratio: thickness ≤ 0.5 mm AND width ≥ 5×thickness) on a planar
//   face is a label candidate.  Full geometric matching of thin slabs is
//   harder than spot_weld discs because the slab presents 6 planar faces
//   that easily merge with the base — we rely primarily on metadata.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace label_apply {

constexpr const char* kSkillId = "label_apply";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm       = 0.0;   // label CENTRE in world XY
    double      position_y_mm       = 0.0;
    double      label_width_mm      = 0.0;
    double      label_height_mm     = 0.0;
    double      label_thickness_mm  = 0.1;   // default 100 µm sticker
    std::string label_text          = "";    // optional printed text on label
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace label_apply
}  // namespace koocadcam::skill
