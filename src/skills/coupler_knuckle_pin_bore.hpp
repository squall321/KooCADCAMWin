#pragma once
// @lat: [[engine/skills#coupler_knuckle_pin_bore]]
//
// coupler_knuckle_pin_bore — Railway coupler knuckle pivot-pin bore (slice 16,
// railway / rolling stock).
//
// An AAR-type coupler knuckle pivots on a vertical pin.  The casting is
// machined with three features about the pin axis:
//   1. pivot pin bore — H7 hole basis (running fit on the pin) via
//      _iso286_fits.hpp h7_max_mm.
//   2. thrust-washer recess — a wider, shallow counterbore at the top that
//      seats the thrust washer.
//   3. transverse lock-pin cross hole — a horizontal hole through the
//      knuckle that captures the lock pin.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. pivot pin bore (cylinder cut DOWN the pin axis, through)
//   2. thrust-washer recess counterbore (wider cylinder at top)
//   3. transverse lock-pin cross hole (radial cylinder, non-overlapping
//      with the recess Z zone)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT   : every dimension must be > 0.
//   DFM-RECESS  : washer_recess_dia_mm must exceed pin_dia_mm.
//   DFM-LOCKPIN : lock_pin_dia_mm must be smaller than pin_dia_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace coupler_knuckle_pin_bore {

constexpr const char* kSkillId = "coupler_knuckle_pin_bore";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy               { 0.0, 0.0, 0.0 };
    double    pin_dia_mm              = 50.0;   // pivot pin Ø (H7 bore)
    double    washer_recess_dia_mm    = 70.0;   // thrust-washer counterbore Ø
    double    washer_recess_depth_mm  = 6.0;    // counterbore depth
    double    lock_pin_dia_mm         = 12.0;   // transverse lock pin Ø
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace coupler_knuckle_pin_bore
}  // namespace koocadcam::skill
