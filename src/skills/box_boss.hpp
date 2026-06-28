#pragma once
// @lat: [[engine/skills#box_boss]]
//
// box_boss — a rectangular (box) boss fused onto a planar face.
//
// The natural feature for a watch LUG, a phone CAMERA ISLAND bump, or any raised
// rectangular pad on a flat OR side surface.  Built with pr::box fused along the
// face's outward normal, so it presents a free outer cap + 4 side walls; its
// inner face merges into the host body (no second free cap) — which is exactly
// why extrude_boss_from_sketch (a congruent-anti-parallel-cap-pair prism
// detector) cannot recover it, and why this dedicated boss skill is needed.
//
//   apply():  resolve entry_face -> origin + outward normal + in-plane axes ->
//             pr::box at the face-local centre -> pr::fuse onto the workpiece.
//
// Recognise (geometric, foreign): a small cluster of planar faces forming a
// rectangular cap (one outer face whose normal = the host face normal) bounded
// by 4 side-wall planes, all PROTRUDING from the host (the cap sits OUTSIDE the
// host surface).  Recover length/width/height from the cap + wall extents.
//
// DFM:
//   - length_mm > 0, width_mm > 0, height_mm > 0

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace box_boss {

constexpr const char* kSkillId = "box_boss";

struct Input
{
    FaceDatum entry_face;             // planar face the boss sits on
    double    center_x_mm = 0.0;      // boss centre in face-local XY
    double    center_y_mm = 0.0;
    double    length_mm   = 0.0;      // along the face-local X axis
    double    width_mm    = 0.0;      // along the face-local Y axis
    double    height_mm   = 0.0;      // rise along the outward face normal

    // Direct world placement (the foreign-recovery path).  When `use_world` is
    // set, the boss is placed at `world_*` with outward direction `world_normal`
    // WITHOUT resolving entry_face — needed for a lug fused onto a CURVED host
    // (a watch case side) where there is no planar datum face to resolve.
    bool      use_world    = false;
    double    world_cx_mm = 0.0, world_cy_mm = 0.0, world_cz_mm = 0.0;  // cap-face centre
    double    world_nx = 0.0, world_ny = 0.0, world_nz = 1.0;          // outward normal
    // In-plane LENGTH direction (along which length_mm runs); zero => apply picks
    // an arbitrary in-plane axis (fine for a square or when orientation is moot).
    double    world_xx = 0.0, world_xy = 0.0, world_xz = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace box_boss
}  // namespace koocadcam::skill
