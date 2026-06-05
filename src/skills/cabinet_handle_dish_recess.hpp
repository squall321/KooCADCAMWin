#pragma once
// @lat: [[engine/skills#cabinet_handle_dish_recess]]
//
// cabinet_handle_dish_recess — Recessed bar-handle dish for a flight-case /
// loudspeaker cabinet (slice 16, pro audio hardware).
//
// Touring cabinets use recessed "dish" handles: a rectangular pocket sunk
// into the panel houses a spring bar handle, with two bolt holes through the
// pocket floor securing the handle hardware.  This skill machines that from
// a flat panel (top) face:
//   1. recessed dish pocket (rounded-rect box pocket, NOT through)
//   2..3. two bolt mounting holes through the pocket floor at bolt_spacing
//
// `bolt_thread_key` selects the bolt clearance diameter from the central
// metric / UNC-UNF thread tables.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. dish pocket          (rounded-rect pocket cut DOWN from top face)
//   2..3. two bolt holes    (cylinders cut through)
//
// subfeature_count = 1 + 2 = 3.
//
// DFM:
//   DFM-INPUT   : dish dims and bolt spacing must be > 0.
//   DFM-THREAD  : bolt_thread_key must resolve in the central thread table.
//   DFM-BOLTFIT : bolt holes (at +/- spacing/2) must fit inside the dish.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cabinet_handle_dish_recess {

constexpr const char* kSkillId = "cabinet_handle_dish_recess";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy       { 0.0, 0.0, 0.0 };
    double      dish_length_mm  { 0.0 };   // pocket length (X)
    double      dish_width_mm   { 0.0 };   // pocket width  (Y)
    double      dish_depth_mm   { 0.0 };   // pocket depth (blind)
    double      bolt_spacing_mm { 0.0 };   // bolt-to-bolt centre distance (X)
    std::string bolt_thread_key;           // "M6" etc. (central table)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cabinet_handle_dish_recess
}  // namespace koocadcam::skill
