#pragma once
// @lat: [[engine/skills#lockbolt_pin_hole_compound]]
//
// lockbolt_pin_hole_compound — COMPOUND aerospace Lockbolt (Huck) pin + collar
// hole.  Two-piece swage fastener: a grooved pin is inserted through the
// joint, a collar is swaged onto the pin grooves under high force, then the
// pintail is broken off.  Common in aircraft structural joints (NAS1396 /
// MIL-B-23086).
//
//   sub-feature 1: straight through-bore at pin_dia (close clearance)
//   sub-feature 2: small collar-side relief counterbore (1 mm deep × +0.4 mm
//                  diameter) so the swaged collar lands cleanly on the
//                  surface and avoids stress at the bore edge.
//
// Sequential pr::cut(wp, cutter) loop (slice-14 guard).
//
// Signature:
//   pattern.kind                   = "lockbolt_pin_hole_compound"
//   pattern.is_compound            = true
//   pattern.aerospace_feature_type = "lockbolt_pin_hole"
//   pattern.fastener_dia           = pin_dia_mm
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   DFM-INPUT          : all dimensions > 0
//   DFM-PIN-DIA        : pin_dia ∈ {4.78, 6.35, 7.94} mm (3/16, 1/4, 5/16)
//   DFM-GRIP-LENGTH    : grip_length ≥ pin_dia × 0.6 (min stack-up)
//   DFM-COLLAR-CLEAR   : collar_clearance_mm > 0 and < pin_dia × 0.25

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lockbolt_pin_hole_compound {

constexpr const char* kSkillId = "lockbolt_pin_hole_compound";

struct Input
{
    int    face_id              = -1;
    double center_x_mm          = 0.0;
    double center_y_mm          = 0.0;
    double pin_dia_mm           = 6.35;   // 4.78 / 6.35 / 7.94
    double grip_length_mm       = 0.0;    // joint stack-up thickness
    double collar_clearance_mm  = 0.2;    // collar-side relief radial clearance
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lockbolt_pin_hole_compound
}  // namespace koocadcam::skill
