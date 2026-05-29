#pragma once
// @lat: [[engine/skills#jig_plate_with_drill_bushings]]
//
// jig_plate_with_drill_bushings — compound fixturing feature.
//
// A jig plate is a hardened rectangular tool steel plate whose job is to
// guide a drill bit precisely.  Hardened drill bushings are press-fitted
// into bored holes; the bit then runs through the bushing into the part.
//
// Per ASME Y14.5 / DIN 179 (Drill bushings — bored type), each bushing
// site is a *triple* feature:
//
//   1. PILOT  — clearance hole through the plate (Ø = bushing OD)
//   2. SEAT   — counterbore (Ø = bushing flange OD), goes flange_depth
//               into the entry face so the flange sits flush.
//   3. CHAMFER — 45° lead-in on entry (eases bushing press-in).
//
// All three sub-features share the same axis; per programmed (x, y) site
// the cutter is: fuse(pilot, seat, chamfer-cone) then a single Boolean
// subtraction from the plate.  Sites are processed sequentially; the
// chain length is therefore 3 × N sub-cuts (N = bushing count).
//
// DFM:
//   DIN 179 part 1 — bushing OD ≥ 2 × clearance bore Ø + 1 mm wall
//   ASME Y14.5     — flange clearance to plate edge ≥ flange OD/2 + 2 mm
//   DFM-002        — clearance bore Ø ≥ 0.8 mm (smallest carbide drill)
//   center-to-center spacing ≥ flange OD + 1 mm (no overlapping seats)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace jig_plate_with_drill_bushings {

constexpr const char* kSkillId = "jig_plate_with_drill_bushings";

struct BushingSite
{
    double x_mm = 0.0;
    double y_mm = 0.0;
};

struct Input
{
    FaceDatum                entry_face;
    gp_Dir                   axis_dir            { 0.0, 0.0, -1.0 };
    double                   bushing_id_mm       = 5.0;   // bore through plate (= bit clearance)
    double                   bushing_od_mm       = 10.0;  // press-fit Ø in plate (counterbore)
    double                   flange_od_mm        = 13.0;  // flange seat at entry
    double                   flange_depth_mm     = 3.0;   // counterbore depth at entry
    double                   chamfer_mm          = 0.5;   // 45° lead-in size
    std::vector<BushingSite> sites;                       // ≥ 1 position
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace jig_plate_with_drill_bushings
}  // namespace koocadcam::skill
