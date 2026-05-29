#pragma once
// @lat: [[engine/skills#linear_slider_track]]
//
// linear_slider_track — motion-defining compound feature.
//
// Builds a linear sliding-track pocket compound, used as the canonical
// mechanism geometry for sliding-lid latches, drawer-rail joints, and
// linear key-locator features in metal-front mechanical assemblies.
//
// COMPOUND CHAIN (≥ 2 primitive cuts/fuses):
//   1. central rectangular SLOT cut (the slider channel)
//   2. retention RIB (fused box at slot floor) — keeps the slider captive
//   3. two END-STOP walls at slot extremities (left in place by clipping)
//   4. two LIMIT walls (the rectangular slot sides) — implicit in cut #1
//
// Equivalent to: cut(rect_slot) ∪ fuse(retention_rib).  We model this as
// (1) cut the main slot, then (2) cut a SHORTER inner slot that leaves the
// retention rib intact at one end, then (3) fuse a small rib block back at
// the slot floor so that it forms a tongue along the centerline.
//
// Signature pattern.kind = "linear_slider_track", is_compound = true,
//                  subfeature_count = 5, spec key = track_class.
//
// Spec table (track_class → standard fits, see ASME-Y14.5 sliding fits H7/g6):
//   "PRECISION_H7"   : slot_clearance 0.020 mm, rib_h 1.5 mm
//   "GENERAL_H8"     : slot_clearance 0.040 mm, rib_h 1.0 mm
//   "LOOSE_H9"       : slot_clearance 0.080 mm, rib_h 0.8 mm
//
// DFM gates:
//   DFM-002  : slot_width  ≥ 0.8 mm (endmill min)
//   DFM-RIB  : rib_height  ≥ 0.5 mm (else rib too weak)
//   DFM-STOP : end_stop_thk ≥ 1.0 mm (compression strength)
//   DFM-SPEC : track_class must be in the spec table

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace linear_slider_track {

constexpr const char* kSkillId = "linear_slider_track";

struct Input
{
    FaceDatum    entry_face;
    double       start_x_mm      = 0.0;
    double       start_y_mm      = 0.0;
    double       end_x_mm        = 0.0;
    double       end_y_mm        = 0.0;
    gp_Dir       axis_dir        { 0.0, 0.0, -1.0 };
    double       slot_width_mm   = 0.0;      // width of the main slider channel
    double       slot_depth_mm   = 0.0;      // depth of the channel below entry
    double       end_stop_thk_mm = 0.0;      // extra material left at each end
    std::string  track_class     = "GENERAL_H8";  // spec key (see table)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Spec-table lookup (exposed for tests).
struct TrackSpec {
    const char* key;
    double      slot_clearance_mm;
    double      rib_height_mm;
};
const TrackSpec* lookupTrackSpec(const std::string& key);

}  // namespace linear_slider_track
}  // namespace koocadcam::skill
