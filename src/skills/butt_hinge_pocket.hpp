#pragma once
// @lat: [[engine/skills#butt_hinge_pocket]]
//
// butt_hinge_pocket — COMPOUND HINGE FEATURE.
//
// A butt hinge (door / cabinet / hardware) mortises into the doorframe as
// a rectangular leaf-recess of standardised size (hinge_length × hinge_width
// × leaf_thickness).  The pocket is paired with TWO screw counterbores
// (one near each end of the leaf) and a leaf-clearance chamfer along the
// inner long edge so the leaf can swing past flush.
//
// Sub-feature chain (3 primitive cuts):
//   1. rectangular pocket           (depth = leaf thickness, 1.5 mm by default)
//   2. screw counterbore × 2        (pilot + seat at 1/4 and 3/4 of length)
//   3. leaf-clearance edge chamfer  (small triangular prism along the long
//                                    pivot-side edge; relieves swing
//                                    interference)
//
//                  long pivot-side edge ─ chamfer prism
//                  ╲
//             ┌─────●─────────────────●────┐
//             │  ◎                    ◎  │     ◎ = counterbore screw holes
//             │                            │     (1/4 L  and  3/4 L)
//             │     ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒     │
//             │     ▒  leaf pocket  ▒     │
//             │     ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒     │
//             └────────────────────────────┘
//
// Standard (DIN 1052 / typical residential):
//   - leaf thickness  : 1.5 – 2.5 mm  → pocket depth
//   - screw size      : #6 wood / M3.5 metric  → seat 7.5 mm, pilot 3.5 mm
//   - hinge_length    : 50 – 100 mm
//   - hinge_width     : 25 – 40 mm
//
// DFM gates:
//   DFM-INPUT          : hinge_length, hinge_width > 0
//   DFM-HINGE-LENGTH   : 30 mm ≤ hinge_length ≤ 150 mm (catalog range)
//   DFM-HINGE-WIDTH    : 15 mm ≤ hinge_width ≤ 60 mm
//   DFM-HINGE-FRAME    : pocket must fit inside frame (entry face area
//                        must be ≥ hinge_length × hinge_width + margin).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace butt_hinge_pocket {

constexpr const char* kSkillId = "butt_hinge_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm      = 0.0;
    double      center_y_mm      = 0.0;
    gp_Dir      axis_dir         { 0.0, 0.0, -1.0 };
    double      hinge_length_mm  = 0.0;   // X extent (along door edge)
    double      hinge_width_mm   = 0.0;   // Y extent (into frame)
    double      leaf_thickness_mm = 1.5;  // pocket depth (default 1.5 mm)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace butt_hinge_pocket
}  // namespace koocadcam::skill
