#pragma once
// @lat: [[engine/skills#rack_ear_19inch_eia]]
//
// rack_ear_19inch_eia — 19-inch rack ear, EIA universal hole pattern
// (slice 16, pro audio hardware).
//
// EIA-310 (and the matching universal/server-rack pattern) defines the
// vertical hole spacing within a rack unit (1U) as a repeating group of
// three holes spaced 12.7 mm / 15.875 mm / 12.7 mm (0.500 / 0.625 / 0.500
// inch).  This skill machines a mounting ear from a flat front face:
//   1..3. three vertical mounting holes at EIA spacing
//   4.    a horizontal handle slot (rounded-end rectangular pocket)
//
// Optional `hole_thread_key` selects a clearance diameter from the central
// thread tables (e.g. "M6" or a UNC key); when empty, `hole_dia_mm` is used
// verbatim.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1..3. three EIA mounting holes (cylinders cut DOWN from front face)
//   4.    handle slot (rounded-rect pocket)
//
// subfeature_count = 3 + 1 = 4.
//
// DFM:
//   DFM-INPUT   : hole_dia, handle slot dims must be > 0.
//   DFM-THREAD  : if hole_thread_key non-empty it must resolve in the
//                 central metric OR UNC/UNF table.
//   DFM-SLOT    : handle slot length must exceed its width.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rack_ear_19inch_eia {

constexpr const char* kSkillId = "rack_ear_19inch_eia";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy           { 0.0, 0.0, 0.0 };   // center of 3-hole group
    double      hole_dia_mm         { 0.0 };             // used if no thread key
    std::string hole_thread_key;                         // optional "M6" etc.
    double      handle_slot_len_mm  { 0.0 };
    double      handle_slot_wid_mm  { 0.0 };
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rack_ear_19inch_eia
}  // namespace koocadcam::skill
