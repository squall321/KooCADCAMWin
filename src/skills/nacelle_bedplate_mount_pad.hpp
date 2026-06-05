#pragma once
// @lat: [[engine/skills#nacelle_bedplate_mount_pad]]
//
// nacelle_bedplate_mount_pad — Wind turbine nacelle bedplate mounting pad
// (slice 16, wind domain).
//
// Drivetrain components (gearbox, generator, main-bearing housing) bolt to
// raised, machined mounting pads cast into the nacelle bedplate.  Each pad is
// a flat raised boss with a 4-bolt pattern drilled through it.  This skill
// FUSES the raised pad onto the top face, then DRILLS 4 bolt holes through
// the pad — net material is positive (pad volume >> 4 small holes).
//
// Sub-features (SEQUENTIAL pr::fuse then pr::cut, no compound boolean):
//   1     : raised machined mounting pad (fused boss box)
//   2..5  : 4 bolt holes drilled through the pad (sequential cut)
//
// subfeature_count = 1 + 4 = 5.   ADDITIVE (net volume positive).
//
// DFM:
//   DFM-INPUT    : every dimension must be > 0.
//   DFM-THREAD   : bolt_thread_key must exist in central metric thread table.
//   DFM-SPACING  : bolt spacing must fit inside the pad with edge clearance.
//   DFM-NET      : pad volume must exceed the 4 bolt holes (net positive).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace nacelle_bedplate_mount_pad {

constexpr const char* kSkillId = "nacelle_bedplate_mount_pad";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy        { 0.0, 0.0, 0.0 };
    double      pad_length_mm    = 200.0;
    double      pad_width_mm     = 150.0;
    double      pad_height_mm    = 30.0;
    double      bolt_spacing_x_mm = 140.0;
    double      bolt_spacing_y_mm = 100.0;
    std::string bolt_thread_key  = "M20";   // M-thread in _iso_thread_table
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace nacelle_bedplate_mount_pad
}  // namespace koocadcam::skill
