#pragma once
// @lat: [[engine/skills#coil_spring_seat_compound]]
//
// coil_spring_seat_compound — COMPOUND FEATURE (3 primitive sub-cuts).
//
// Realistic coil-spring mounting pocket on a metal frame: a cylindrical
// pocket sized for a compression spring's outside diameter (slip fit) +
// a flat shoulder at the bottom (so the spring's compressed coils block
// solidly) + a center pilot pin hole (locates the spring concentrically).
//
//   Sub-feature chain:
//     1. Spring pocket cylinder  (radius = (spring_od + slip_fit) / 2,
//                                 depth  = free_height + 1 mm head room)
//     2. Pilot pin hole          (radius = pilot_dia / 2,
//                                 depth  = free_height + 4 mm engagement)
//     3. Implicit annular shoulder at the spring-pocket floor — the
//                                 pilot hole goes DEEPER than the spring
//                                 pocket, leaving a planar annulus at the
//                                 pocket floor that the compressed coils
//                                 sit on.  This is the geometric
//                                 by-product of (1) + (2) with two
//                                 different depths, NOT an extra cut.
//
//   The two concentric overlapping cylinders are FUSED first, then a
//   single Boolean cut against the workpiece — matches the
//   "fuse_first_then_cut" pattern from counterbore.cpp.
//
// Output topology: 2 cylindrical faces (pocket + pilot) + 1 annular
// shoulder (planar) + 1 pilot blind bottom.  Single Workpiece, single
// FeatureSignature with pattern.is_compound = true, subfeature_count = 3.
//
// DFM gates (DIN 2098 compression-spring assembly):
//   - spring_od >= 2 mm and <= 200 mm (covered manufacturing range)
//   - pilot_dia <= 0.7 × spring inside diameter (must clear inner coils;
//     conservatively pilot_dia <= 0.5 × spring_od)
//   - pilot_dia >= 0.8 mm (DFM-002 minimum drill)
//   - free_height >= 3 × spring_od / 5 (avoid block-coil pocket)
//   - workpiece thickness must accommodate (pilot depth + 1 mm wall)
//
// Recognize: metadata replay (compound spring seat is hard to
// fingerprint geometrically — many features leave concentric cyls +
// annular shoulder).  Confidence 1.0 from history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace coil_spring_seat_compound {

constexpr const char* kSkillId = "coil_spring_seat_compound";

struct Input
{
    FaceDatum   face_id;                       // entry face
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      spring_od       = 0.0;         // spring outside diameter, mm
    double      free_height     = 0.0;         // uncompressed spring height, mm
    double      pilot_dia       = 0.0;         // pilot pin diameter, mm
    double      slip_fit_mm     = 0.2;         // radial clearance for spring OD
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace coil_spring_seat_compound
}  // namespace koocadcam::skill
