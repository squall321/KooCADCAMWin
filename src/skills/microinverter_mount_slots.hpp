#pragma once
// @lat: [[engine/skills#microinverter_mount_slots]]
//
// microinverter_mount_slots — Microinverter / power-optimizer mounting bracket
// (slice 16, solar / PV mounting).
//
// Module-level power electronics (MLPE) bolt to the rail through elongated
// slots that tolerate rail-spacing variation.  Each slot is an obround
// (stadium) shape: a central box with a half-round cylinder at each end.  A
// cable strain-relief notch routes the trunk cable.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   slot 0: box + end cyl + end cyl   (3 cuts)
//   slot 1: box + end cyl + end cyl   (3 cuts)
//   notch:  box                       (1 cut)
//
// subfeature_count = 2*3 + 1 = 7.
//
// DFM:
//   DFM-INPUT    : every dimension must be > 0.
//   DFM-SLOTGEOM : slot_length_mm must be > slot_width_mm (else it is a hole).
//   DFM-SPACING  : slot_spacing_mm must be > slot_width_mm (slots overlap).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace microinverter_mount_slots {

constexpr const char* kSkillId = "microinverter_mount_slots";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy       { 0.0, 0.0, 0.0 };
    double    slot_length_mm  = 20.0;
    double    slot_width_mm    = 7.0;
    double    slot_spacing_mm  = 30.0;
    double    notch_width_mm   = 8.0;
    double    notch_depth_mm   = 6.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace microinverter_mount_slots
}  // namespace koocadcam::skill
