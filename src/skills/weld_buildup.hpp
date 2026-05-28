#pragma once
// @lat: [[engine/skills#weld_buildup]]
//
// weld_buildup — additive cladding / repair.  Deposits a thin slab of weld
// metal on an existing planar face to restore worn geometry or add local
// thickness (e.g. tool/die repair, wear-surface restoration).
//
//                                                              ▲
//                                                              │ buildup_thickness_mm
//      ┌─────── entry_face (worn face) ───────┐                ▼
//      │                                       │   ─►  ┌──────────────────────┐
//      │                                       │       │░░░░░░ weld bead ░░░░░│  ← new slab
//      │                                       │       ├──────────────────────┤
//      │            workpiece                  │       │     (parent face)    │
//      │                                       │       │      workpiece       │
//      └───────────────────────────────────────┘       └──────────────────────┘
//
// vs spot_weld: spot_weld fuses a small disc bump (point-join two parts);
//                weld_buildup deposits an EXTENSIVE slab covering the whole
//                planar face to restore lost thickness.
// vs seam_weld:  seam_weld lays a long thin bead along a line;
//                weld_buildup fills a 2-D face area.
//
// Geometry: face_milling in reverse — fetch the entry face's planar bbox,
// extrude that footprint along the outward normal by buildup_thickness_mm,
// fuse onto the parent workpiece (sink 1% into parent for robust fuse).
//
// Signature pattern:
//   pattern.kind == "weld_buildup"
//   pattern.thickness_mm
//   pattern.material
//   pattern.target_face_id   (the face id at apply-time, for reference)
//   pattern.entry_face_normal
//   pattern.additive == true
//
// DFM:
//   DFM-BUILDUP-THICK : buildup_thickness_mm ∈ [0.5, 10.0]
//                       <0.5 mm = below practical bead height (single pass)
//                       >10  mm = needs multi-pass + post heat treatment
//                       (and is more economically machined from solid stock)
//
// Recognize:
//   A thin slab adjacent to a planar face whose normal matches the slab's
//   outward direction; thickness < 10 mm.  Slice-1: replay history first;
//   fall back to detecting small planar protrusions matching the
//   bbox of the parent face's footprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace weld_buildup {

constexpr const char* kSkillId = "weld_buildup";

struct Input
{
    FaceDatum     entry_face;                        // the worn face to clad
    double        buildup_thickness_mm   = 0.0;
    std::string   material               = "stainless_steel";
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace weld_buildup
}  // namespace koocadcam::skill
