#pragma once
// @lat: [[engine/skills#counterbore_ring_pattern]]
//
// counterbore_ring_pattern — GENERATIVE side of the counterbored-bolt-circle
// grammar compound.
//
// A ring of N identical counterbores (a screw-head seat ring — watch casebacks,
// flanged covers) was the measured worst case of the RE pattern pipeline: the
// grammar recovered the pilot ring and the seat ring as TWO separate
// bolt_circle_pattern candidates whose replay left every pilot ~seat-depth
// short (~18 % removed-volume error), while dedupe face arbitration blocked the
// individual counterbore candidates entirely.  This compound is the missing
// SINGLE explanation: one editable step that regenerates the whole ring by
// composing counterbore::apply per instance.
//
// Recognition lives in the grammar layer (re::recognizeCompounds ->
// matchCounterboreRings): it fits recognised counterbore candidates' centres to
// a circle and emits this skill_id with the union of ALL member face ids, so
// dedupe's strict-superset rule collapses the pilot-ring / seat-ring
// bolt_circle_pattern duplicates AND the member counterbores into this one
// step.  recognize() here returns empty so the registry does not double-count.
//
// Scope: vertical (±Z) rings — counterbore::Input carries only an (x, y)
// position, and the planar grammar matchers share the same scope.  A side
// (radial-axis) counterbored ring is documented follow-up work.
//
// CAM contract: apply() stamps the compound signature with the resolved entry
// plane `position_z_mm` (the FIRST member counterbore's entry point — the ring
// shares one flat entry face) and `hole_centers` (one [x, y, z] entry point per
// member), so a replayed workpiece that carries ONLY this compound signature is
// still machinable: cam::counterboreRingPatternToolpath regenerates the
// per-instance seat + pilot plunges from these params instead of Z=0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace counterbore_ring_pattern {

constexpr const char* kSkillId = "counterbore_ring_pattern";

struct Input
{
    FaceDatum entry_face = FaceByNormal{ gp_Dir(0.0, 0.0, 1.0) };
    int       count               = 0;      // >= 3 counterbores on the ring
    double    bolt_circle_dia_mm  = 0.0;    // pitch-circle diameter
    double    center_x_mm         = 0.0;
    double    center_y_mm         = 0.0;
    gp_Dir    axis_dir            { 0.0, 0.0, -1.0 };  // drilling direction (±Z)
    double    pilot_dia_mm        = 0.0;    // through/deep small bore
    double    pilot_depth_mm      = 0.0;    // from the entry face
    double    seat_dia_mm         = 0.0;    // screw-head seat
    double    seat_depth_mm       = 0.0;    // from the entry face (< pilot depth)
    double    start_angle_deg     = 0.0;    // angle of the first instance (0 = +X)
};

// Synthesis: cut the ring; returns the new workpiece + the compound signature.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation (count, diameters, depths, pitch clearance, axis).
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognition is performed by re::recognizeCompounds (koo_re); empty here.
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace counterbore_ring_pattern
}  // namespace koocadcam::skill
