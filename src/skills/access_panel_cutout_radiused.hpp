#pragma once
// @lat: [[engine/skills#access_panel_cutout_radiused]]
//
// access_panel_cutout_radiused — Access-panel cutout with radiused corners
// plus a perimeter nutplate rivet pattern (aerospace structures).
//
// Inspection/access panels in skins and ribs are rectangular cutouts with
// generously radiused corners (to avoid crack initiation) and a ring of
// nutplate rivet holes around the perimeter where the cover plate's nutplates
// are riveted on.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. rounded-rectangle panel cutout (box, then 4 corner-radius cylinders cut)
//   N. perimeter nutplate rivet holes (one cylinder each, sequential)
//   subfeature_count = 1 + nutplate_count.
//
// DFM:
//   DFM-INPUT  : every dimension/count must be > 0.
//   DFM-CORNER : corner_radius_mm must be < min(panel_w, panel_h)/2.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace access_panel_cutout_radiused {

constexpr const char* kSkillId = "access_panel_cutout_radiused";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy        { 0.0, 0.0, 0.0 };   // panel centre (XY)
    double    panel_w_mm       { 60.0 };
    double    panel_h_mm       { 40.0 };
    double    corner_radius_mm { 6.0 };
    int       nutplate_count   { 8 };
    double    rivet_dia_mm     { 3.2 };
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace access_panel_cutout_radiused
}  // namespace koocadcam::skill
