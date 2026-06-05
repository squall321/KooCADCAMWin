#pragma once
// @lat: [[engine/skills#cam_lock_minifix_bore]]
//
// cam_lock_minifix_bore — Cam-lock (Minifix-style) knock-down joint
// (slice 16, furniture / cabinetry hardware).
//
// Minifix cam-and-dowel joints are the workhorse of flat-pack furniture: a
// ~15 mm cam housing is pressed into a face bore, a connecting bolt enters
// from the adjoining panel through a transverse cross hole and engages the
// cam, and a wood dowel alongside locates the joint.  This skill machines
// that pattern from a flat (top) face:
//   1. cam housing bore   (blind 15 mm cylinder cut DOWN from top face)
//   2. dowel bore         (blind cylinder offset from the cam, cut DOWN)
//   3. connecting-bolt cross hole (transverse cylinder along Y into the cam)
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. cam housing bore  (vertical cylinder)
//   2. dowel bore        (vertical cylinder, non-overlapping)
//   3. bolt cross hole   (horizontal cylinder; may intersect cam — cut last,
//                         sequentially, never via compound boolean)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT   : all dimensions must be > 0.
//   DFM-CAMDIA  : cam_dia_mm must be in [10, 25].
//   DFM-DOWELFIT: dowel bore (at dowel_offset) must clear the cam wall.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cam_lock_minifix_bore {

constexpr const char* kSkillId = "cam_lock_minifix_bore";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy        { 0.0, 0.0, 0.0 };
    double      cam_dia_mm       { 15.0 };   // cam housing bore
    double      cam_depth_mm     { 12.5 };   // cam housing depth (blind)
    double      dowel_dia_mm     { 8.0 };    // wood dowel bore
    double      dowel_offset_mm  { 32.0 };   // cam-to-dowel centre (X)
    double      bolt_dia_mm      { 7.0 };    // connecting-bolt cross hole
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cam_lock_minifix_bore
}  // namespace koocadcam::skill
