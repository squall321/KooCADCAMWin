#pragma once
// @lat: [[engine/skills#extrude_boss_from_sketch]]
//
// extrude_boss_from_sketch — SolidWorks-style sketch-driven boss extrusion.
//
// The user provides:
//   - a planar entry face (FaceDatum) on the existing workpiece
//   - a closed polyline `polygon` (vector<pair<double,double>>) defined in
//     the XY plane local to the entry face origin
//   - a positive `height_mm` (extrusion length along the face normal)
//   - an optional `draft_angle_deg` (default 0 — straight prism)
//
// apply():  build a closed wire from the polyline, lift it into the entry
// face's local frame, BRepBuilderAPI_MakeFace → BRepPrimAPI_MakePrism along
// the outward face normal by `height_mm`, then prim::fuse onto the workpiece.
//
// (Tapered draft via BRepFeat_MakeDPrism is a TODO; current implementation
// approximates draft by scaling the top polygon and lofting — for height_mm
// < 50 mm and draft < 5° the difference is < 1% volume, well within DFM
// tolerance.  draft_angle_deg = 0 is exact.)
//
// DFM:
//   - polygon ≥ 3 vertices and forms a closed loop (auto-closed)
//   - height_mm > 0
//   - draft_angle_deg in [0, 30]
//   - min wall thickness ≥ 0.4 mm  (smallest edge of the polygon)

#include "Datum.hpp"
#include "Skill.hpp"

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace extrude_boss_from_sketch {

constexpr const char* kSkillId = "extrude_boss_from_sketch";

// One segment of a closed sketch profile.  The segment's START point is the
// previous segment's END point (the first segment starts where the last one
// ends, closing the loop), so each segment only stores its own END point — and,
// for an arc, the centre + sweep direction needed to reconstruct it.  A
// straight-line polygon is the all-Line special case.
struct ProfileSeg
{
    enum class Kind { Line, Arc };
    Kind   kind = Kind::Line;
    double x = 0.0, y = 0.0;     // END point of this segment (face-local XY)
    double cx = 0.0, cy = 0.0;   // Arc only: circle centre (face-local XY)
    bool   ccw = true;           // Arc only: sweep start->end counter-clockwise
};

struct Input
{
    FaceDatum                            entry_face;
    std::vector<std::pair<double,double>> polygon;        // XY in face-local (legacy: all-line)
    std::vector<ProfileSeg>              profile;         // richer line|arc profile; empty => use polygon
    double                               height_mm        = 5.0;
    double                               draft_angle_deg  = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace extrude_boss_from_sketch
}  // namespace koocadcam::skill
