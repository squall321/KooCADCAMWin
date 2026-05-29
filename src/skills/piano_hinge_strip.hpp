#pragma once
// @lat: [[engine/skills#piano_hinge_strip]]
//
// piano_hinge_strip — long strip hinge with N alternating knuckle slots, a
// single pin bore through all knuckles, and 2 × N mounting holes.
//
// REAL geometry chain (≥ 3 cuts):
//   1. N rectangular "knuckle" slots cut down through one half of the strip
//      width (alternating top/bottom relative to the pin axis is modelled as
//      one-sided slots in slice-1).
//   2. 1 transverse pin bore — a cylinder running the entire strip length
//      along the hinge axis, drilled THROUGH every knuckle.
//   3. 2 × N small mounting screw holes (drilled across the strip width on
//      both flanges, evenly distributed between the knuckles).
//
//                                          ◄────── strip_length_mm ──────►
//                          ┌────────┬────────┬────────┬────────┬────────┐
//   strip_width_mm        │   ●    │  slot  │   ●    │  slot  │   ●    │
//   (perpendicular to     │  o  o  │        │  o  o  │        │  o  o  │  ◄── mounting holes
//   pin axis)             └────────┴────────┴────────┴────────┴────────┘
//                                                  ▲
//                          ◄─────── pin bore runs entire length along X ──►
//
// Standard:
//   - ANSI/BHMA A156.1 (commercial piano hinges): typical 25 mm wide,
//     1.5 – 3 mm thick, knuckle pitch 38 mm or 76 mm; pin dia 3 – 4 mm.
//   - DIN 7951 / ISO 7045 mounting screws: M3 / M4 wood/sheet metal.
//
// DFM gates:
//   - knuckle_count ≥ 2 (else not a hinge).
//   - pin_dia_mm    ≥ 0.8 mm (standard small endmill — DFM-002).
//   - knuckle_pitch_mm > 1.5 × pin_dia_mm (else slot wall < 1 wall thickness).
//   - mount_hole_dia ≥ 0.8 mm.
//   - strip_thickness_mm > pin_dia_mm + 0.6 mm (1.5× pin clearance).
//
// Signature:
//   - is_compound = true
//   - subfeature_count = 1 (slot prism, fused N-up) + 1 (pin bore) +
//                        2 × knuckle_count (mounting holes) = 2 + 2N
//
// Recognize:
//   - PRIMARY (geometric): walk faces, look for a single LONG horizontal
//     cylindrical face (the pin bore) running the strip length; count N
//     vertical-walled rectangular slot signatures; count circular
//     mounting-hole edges.  Confidence 0.7 when at least pin bore + 2 slots
//     + 4 mount-holes match.
//   - FALLBACK (metadata replay): feature.skill_id match → confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace piano_hinge_strip {

constexpr const char* kSkillId = "piano_hinge_strip";

struct Input
{
    FaceDatum   entry_face;
    double      origin_x_mm        = 0.0;      // strip origin XY (corner)
    double      origin_y_mm        = 0.0;
    gp_Dir      pin_axis_dir       { 1.0, 0.0, 0.0 };   // pin runs along +X
    double      strip_length_mm    = 0.0;      // along pin_axis_dir
    double      strip_width_mm     = 0.0;      // perpendicular in XY
    double      strip_thickness_mm = 0.0;      // along -Z from entry face
    int         knuckle_count      = 4;
    double      knuckle_pitch_mm   = 25.0;     // center-to-center
    double      knuckle_slot_width_mm = 8.0;
    double      knuckle_slot_depth_mm = 0.0;   // defaults to strip_thickness / 2
    double      pin_dia_mm         = 3.0;
    double      mount_hole_dia_mm  = 3.2;      // ø3.2 clears M3
    double      mount_hole_inset_mm = 4.0;     // from strip edge
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace piano_hinge_strip
}  // namespace koocadcam::skill
