#pragma once
// @lat: [[engine/skills#yaw_brake_caliper_mount]]
//
// yaw_brake_caliper_mount — Wind turbine yaw-brake caliper mounting pattern
// (slice 16, wind domain).
//
// The yaw brake clamps a disc on the nacelle yaw bearing to hold the nacelle
// against the tower under wind load.  The caliper is bolted to the bedplate
// across the disc; this skill machines the 4 caliper bolt holes (two per
// mounting pad either side of the disc) plus the brake-disc clearance slot
// the caliper straddles.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1..4 : 4 caliper bolt holes (2 pads × 2 holes), spaced pad_spacing apart
//   5    : brake-disc clearance slot (box pocket straddled by the caliper)
//
// subfeature_count = 4 + 1 = 5.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-THREAD    : bolt_thread_key must exist in central metric thread table.
//   DFM-SLOT      : disc slot width/depth must be > 0 and slot must fit
//                   between the two bolt pads (slot_width < pad_spacing).
//   DFM-BOLT-FIT  : bolt_dia must clear the thread nominal.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace yaw_brake_caliper_mount {

constexpr const char* kSkillId = "yaw_brake_caliper_mount";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy          { 0.0, 0.0, 0.0 };
    double      pad_spacing_mm     = 120.0;  // across-disc pad-to-pad spacing (X)
    std::string bolt_thread_key    = "M16";  // M-thread in _iso_thread_table
    double      bolt_dia_mm        = 17.0;   // clearance hole dia
    double      disc_slot_width_mm = 40.0;   // disc clearance slot width (X)
    double      disc_slot_depth_mm = 30.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace yaw_brake_caliper_mount
}  // namespace koocadcam::skill
