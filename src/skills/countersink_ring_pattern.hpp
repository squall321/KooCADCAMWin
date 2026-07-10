#pragma once
// @lat: [[engine/skills#countersink_ring_pattern]]
//
// countersink_ring_pattern — GENERATIVE side of the countersunk-bolt-circle
// grammar compound.
//
// A ring of N identical countersinks (a flat-head-screw seat ring — watch
// casebacks, flush cover plates) fails the RE pattern pipeline the same way
// the counterbored ring did: the grammar recovers the pilot ring as a
// bolt_circle_pattern that subsumes the pilot drills yet BLOCKS every member
// countersink candidate (cyl face shared, cone face not contained), so the
// replayed plan loses the whole cone volume (-43.7 % measured).  This
// compound is the missing SINGLE explanation: one editable step that
// regenerates the whole ring by composing countersink::apply per instance.
//
// Recognition lives in the grammar layer (re::recognizeCompounds ->
// matchCountersinkRings): it fits recognised countersink candidates' centres
// to a circle and emits this skill_id with the union of ALL member face ids,
// so dedupe's strict-superset rule collapses the pilot-ring
// bolt_circle_pattern duplicate AND the member countersinks into this one
// step.  recognize() here returns empty so the registry does not double-count.
//
// Scope: vertical (±Z) rings — countersink::Input carries only an (x, y)
// position, and the planar grammar matchers share the same scope.  A side
// (radial-axis) countersunk ring is documented follow-up work.
//
// CAM contract: apply() stamps the compound signature with the resolved entry
// plane `position_z_mm` (the FIRST member countersink's entry point — the ring
// shares one flat entry face), the atom-derived `cone_depth_mm` and
// `hole_centers` (one [x, y, z] entry point per member), so a replayed
// workpiece that carries ONLY this compound signature is still machinable:
// cam::countersinkRingPatternToolpath regenerates the per-instance pilot +
// chamfer plunges from these params instead of Z=0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace countersink_ring_pattern {

constexpr const char* kSkillId = "countersink_ring_pattern";

struct Input
{
    FaceDatum entry_face = FaceByNormal{ gp_Dir(0.0, 0.0, 1.0) };
    int       count               = 0;      // >= 3 countersinks on the ring
    double    bolt_circle_dia_mm  = 0.0;    // pitch-circle diameter
    double    center_x_mm         = 0.0;
    double    center_y_mm         = 0.0;
    gp_Dir    axis_dir            { 0.0, 0.0, -1.0 };  // drilling direction (±Z)
    double    pilot_dia_mm        = 0.0;    // cylindrical bore diameter
    double    pilot_depth_mm      = 0.0;    // from the entry face
    double    cone_top_dia_mm     = 0.0;    // large diameter at the entry face
    double    cone_angle_deg      = 90.0;   // INCLUDED angle (82° / 90° / 100° / 120°)
    double    start_angle_deg     = 0.0;    // angle of the first instance (0 = +X)
};

// Synthesis: cut the ring; returns the new workpiece + the compound signature.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation (count, diameters, cone angle/depth, pitch clearance, axis).
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognition is performed by re::recognizeCompounds (koo_re); empty here.
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace countersink_ring_pattern
}  // namespace koocadcam::skill
