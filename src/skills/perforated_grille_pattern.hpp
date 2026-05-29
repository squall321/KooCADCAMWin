#pragma once
// @lat: [[engine/skills#perforated_grille_pattern]]
//
// perforated_grille_pattern — COMPOUND, SPEC-DRIVEN feature
//
// Cuts a hexagonal array of through holes in a face, forming a perforated
// grille typical of speaker covers, electronics enclosures, and HVAC vents.
//
//   ○ ○ ○ ○ ○         hex pattern: every other row offset by hex_pitch/2
//    ○ ○ ○ ○ ○        rows separated by hex_pitch × √3/2
//   ○ ○ ○ ○ ○
//    ○ ○ ○ ○ ○
//
// Spec library (industry perforation series):
//   "HEX-3" : hole_dia=3 mm, hex_pitch=5 mm  (fine — speaker grille)
//   "HEX-5" : hole_dia=5 mm, hex_pitch=8 mm  (medium — equipment vent)
//   "HEX-8" : hole_dia=8 mm, hex_pitch=12 mm (coarse — HVAC return)
//
// Compound chain:
//   compute hex-grid points in rectangular footprint
//   for each (x, y):   build_cylinder_cutter(x, y, through_depth)
//   cutMany            → single Boolean cut against compound of cylinders
//
// DFM gates:
//   DFM-GRILLE-SP    : unknown spec key.
//   DFM-GRILLE-DIA   : hole_dia < 0.8 mm (DFM-002).
//   DFM-GRILLE-FIT   : hole_dia >= hex_pitch (holes would overlap).
//   DFM-GRILLE-FA    : open area / footprint < 50 %.
//   DFM-GRILLE-N     : computed hole_count < 4 (not a useful pattern).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace perforated_grille_pattern {

constexpr const char* kSkillId = "perforated_grille_pattern";

struct Input
{
    FaceDatum   face;
    std::string spec_key;                // "HEX-3" | "HEX-5" | "HEX-8" | ""
    // Optional overrides used when spec_key is empty.
    double      hole_dia_mm       = 0.0;
    double      hex_pitch_mm      = 0.0;
    // Footprint: a rectangular region in the face plane.
    double      footprint_length_mm = 0.0;   // along grid_axis
    double      footprint_width_mm  = 0.0;   // perpendicular
    // Center of the grille footprint.
    double      center_x_mm       = 0.0;
    double      center_y_mm       = 0.0;
    // Grid orientation: hexagons aligned with this direction.  Defaults to +X.
    gp_Dir      grid_axis         { 1.0, 0.0, 0.0 };
    double      wall_thickness_mm = 0.0;     // defaults to 3 mm
};

struct GrilleSpec {
    double hole_dia_mm  = 0.0;
    double hex_pitch_mm = 0.0;
};

GrilleSpec grilleSpecFor(const std::string& key);

// Compute hex-grid point count for a rectangular footprint.
int hexHoleCount(double footprint_length_mm,
                 double footprint_width_mm,
                 double hex_pitch_mm);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace perforated_grille_pattern
}  // namespace koocadcam::skill
