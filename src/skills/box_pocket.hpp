#pragma once
// @lat: [[engine/skills#box_pocket]]
//
// box_pocket — a rectangular (sharp-corner) POCKET cut into a planar face.
//
// The subtractive mirror of box_boss: the natural feature for a phone/watch SIDE
// BUTTON pocket (a rect recess on a ±X/±Y side face), a SIM tray, or a connector
// cutout.  Cut with pr::box along the INWARD (-normal) direction, so it presents
// a planar FLOOR bounded by 3-or-4 INWARD-facing side walls (all sharp-cornered).
//
// Unlike mill_rect_pocket (Z-axis only, and keyed on corner-FILLET cylinders),
// box_pocket handles an ARBITRARY axis and SHARP corners — the two are
// geometrically disjoint (fillet cylinders vs sharp planar-only walls), so they
// complement rather than compete.
//
//   apply():  resolve entry_face -> origin + outward normal + in-plane axes ->
//             pr::box at the face-local centre -> pr::cut INTO the workpiece.
//
// Recognise (geometric, foreign): a planar FLOOR face bounded by 3-or-4
// perpendicular side-wall planes that face INWARD (toward the floor centre), with
// the floor RECESSED below the entry plane.  Recover length/width/depth from the
// floor + wall extents.
//
// DFM:
//   - length_mm > 0, width_mm > 0, depth_mm > 0

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace box_pocket {

constexpr const char* kSkillId = "box_pocket";

struct Input
{
    FaceDatum entry_face;             // planar face the pocket is cut INTO
    double    center_x_mm = 0.0;      // pocket centre in face-local XY
    double    center_y_mm = 0.0;
    double    length_mm   = 0.0;      // along the face-local X axis
    double    width_mm    = 0.0;      // along the face-local Y axis
    double    depth_mm    = 0.0;      // recess INWARD along the -face normal

    // Direct world placement (the foreign-recovery path).  When `use_world` is
    // set, the pocket is cut at `world_*` with entry outward direction
    // `world_normal` WITHOUT resolving entry_face — needed for a side button on a
    // face that has no resolvable design datum after a STEP round-trip.
    // world_c* is the MOUTH centre (on the entry face); the floor sits depth
    // inboard, mirroring box_boss's cap-centre convention.
    bool      use_world    = false;
    double    world_cx_mm = 0.0, world_cy_mm = 0.0, world_cz_mm = 0.0;  // mouth centre
    double    world_nx = 0.0, world_ny = 0.0, world_nz = 1.0;          // entry outward normal
    // In-plane LENGTH direction (along which length_mm runs); zero => apply picks
    // an arbitrary in-plane axis (fine for a square or when orientation is moot).
    double    world_xx = 0.0, world_xy = 0.0, world_xz = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace box_pocket
}  // namespace koocadcam::skill
