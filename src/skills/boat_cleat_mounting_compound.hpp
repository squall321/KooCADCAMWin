#pragma once
// @lat: [[engine/skills#boat_cleat_mounting_compound]]
//
// boat_cleat_mounting_compound — Horn-cleat mounting base (slice 16, marine /
// boat hardware).
//
// A horn cleat (the classic two-horned deck cleat used to belay dock lines)
// bolts down onto a recessed mounting pad so the casting sits flush in the
// deck.  The mounting prep on the deck/backing-plate consists of:
//   1. recessed mounting pad — a shallow rectangular pocket (box cut) the
//      cleat base drops into.
//   2 & 3. two bolt clearance holes drilled through the pad on the cleat's
//      bolt spacing; clearance comes from the central metric thread table.
//   plus a fairlead lead chamfer broken on the pocket lead edge.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. recessed mounting pad (box cut)
//   2. bolt clearance hole #1
//   3. bolt clearance hole #2
//
// subfeature_count = 3 (pad + 2 holes).
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-M-THREAD  : bolt_thread_size_key must exist in central metric table.
//   DFM-MARINE-SPAN : bolt_spacing_mm must be < cleat_length_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace boat_cleat_mounting_compound {

constexpr const char* kSkillId = "boat_cleat_mounting_compound";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy           { 0.0, 0.0, 0.0 };
    double      cleat_length_mm     = 0.0;   // overall length of the cleat base
    double      bolt_spacing_mm     = 0.0;   // centre-to-centre of the 2 bolts
    std::string bolt_thread_size_key;        // from _iso_thread_table.hpp ("M6"…)
    double      pad_depth_mm        = 0.0;   // recess depth of the mounting pad
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace boat_cleat_mounting_compound
}  // namespace koocadcam::skill
