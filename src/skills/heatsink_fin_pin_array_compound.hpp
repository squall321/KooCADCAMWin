#pragma once
// @lat: [[engine/skills#heatsink_fin_pin_array_compound]]
//
// heatsink_fin_pin_array_compound — COMPOUND pin-fin heat sink.
//
// Fuses a 2-D array of cylindrical pins onto a planar base face.  Each pin
// is a vertical cylinder of pin_dia × pin_height standing on the workpiece
// top face.  Two layouts are supported:
//
//   "grid" : fin_count_x × fin_count_y rectangular grid, pitch (px, py).
//   "hex"  : staggered (slice-12 fill_pattern style); odd rows are shifted
//            by px/2; row pitch py = pitch_y_mm (caller responsibility).
//
// Sub-feature chain:
//   build_pin_cylinder × N×M  →  fuseMany onto wp  →  single solid.
//
// DFM gates:
//   DFM-PIN-DIA   : pin_dia_mm <= 0.5  → un-machinable (drill flute breakage).
//   DFM-PIN-HEIGHT: pin_height_mm <= 0 → invalid extrusion.
//   DFM-PIN-AR    : pin_height_mm / pin_dia_mm > 15  → manufacturability cliff
//                   (typical pin-fin AR ≤ 15).
//   DFM-PIN-PITCH : pitch_x < pin_dia + 0.5  → pins overlap or no fin gap.
//   DFM-PIN-N     : fin_count_x < 2 OR fin_count_y < 2 (not an array).
//   DFM-PIN-LAYOUT: pattern not in {"grid","hex"}.
//   DFM-PIN-FIT   : array span > base bbox max dim.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace heatsink_fin_pin_array_compound {

constexpr const char* kSkillId = "heatsink_fin_pin_array_compound";

struct Input
{
    int         face_id        = -1;     // resolved planar host face (top)
    double      fin_pitch_x_mm = 0.0;
    double      fin_pitch_y_mm = 0.0;
    int         fin_count_x    = 0;
    int         fin_count_y    = 0;
    double      pin_dia_mm     = 0.0;
    double      pin_height_mm  = 0.0;
    std::string pattern        = "grid"; // "grid" | "hex"
    // Array origin: bottom-left pin centre on the base face plane.
    double      origin_x_mm    = 0.0;
    double      origin_y_mm    = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace heatsink_fin_pin_array_compound
}  // namespace koocadcam::skill
