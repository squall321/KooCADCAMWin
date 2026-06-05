#pragma once
// @lat: [[engine/skills#shelf_pin_hole_row_32mm]]
//
// shelf_pin_hole_row_32mm — 32 mm-system shelf-pin hole row
// (slice 16, furniture / cabinetry hardware).
//
// Cabinet side panels are line-bored with rows of 5 mm holes on a 32 mm
// pitch (the "System 32" standard) so adjustable shelves can be repositioned.
// This skill machines one linear hole row from a flat (top) face:
//   1..N. N shelf-pin holes spaced at pitch_mm along +X from row_origin
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   each hole = one vertical cylinder cut DOWN from the top face.
//
// subfeature_count = hole_count.
//
// DFM:
//   DFM-INPUT : pin_dia, pitch, depth must be > 0; hole_count >= 2.
//   DFM-PITCH : pitch_mm must exceed pin_dia_mm (holes must not overlap).
//   DFM-DEPTH : hole_depth_mm must be a blind hole (< panel thickness).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shelf_pin_hole_row_32mm {

constexpr const char* kSkillId = "shelf_pin_hole_row_32mm";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      row_origin     { 0.0, 0.0, 0.0 };   // first hole centre
    double      pin_dia_mm     { 5.0 };             // shelf-pin hole dia
    int         hole_count     { 6 };
    double      pitch_mm       { 32.0 };            // System 32 pitch
    double      hole_depth_mm  { 12.0 };            // blind depth
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shelf_pin_hole_row_32mm
}  // namespace koocadcam::skill
