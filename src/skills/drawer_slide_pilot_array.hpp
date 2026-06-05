#pragma once
// @lat: [[engine/skills#drawer_slide_pilot_array]]
//
// drawer_slide_pilot_array — Drawer-slide mounting pilot-hole array (linear)
// (slice 16, furniture / cabinetry hardware).
//
// Ball-bearing drawer slides screw to the cabinet side with a row of evenly
// spaced pilot holes.  This skill machines that linear pilot array from a
// flat (top) face:
//   1..N. N pilot holes spaced at pitch_mm along +X from row_origin
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   each pilot = one vertical cylinder cut DOWN from the top face.
//
// subfeature_count = hole_count.
//
// DFM:
//   DFM-INPUT : hole_dia, pitch, depth must be > 0; hole_count >= 2.
//   DFM-PITCH : pitch_mm must exceed hole_dia_mm (holes must not overlap).
//   DFM-DEPTH : hole_depth_mm must be blind (< panel thickness).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace drawer_slide_pilot_array {

constexpr const char* kSkillId = "drawer_slide_pilot_array";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      row_origin     { 0.0, 0.0, 0.0 };   // first pilot centre
    double      hole_dia_mm    { 3.0 };             // pilot drill dia
    int         hole_count     { 4 };
    double      pitch_mm       { 32.0 };            // pilot-to-pilot spacing
    double      hole_depth_mm  { 10.0 };            // blind depth
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drawer_slide_pilot_array
}  // namespace koocadcam::skill
