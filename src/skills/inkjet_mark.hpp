#pragma once
// @lat: [[engine/skills#inkjet_mark]]
//
// inkjet_mark — drop-on-demand ink printed onto a workpiece surface.  Used
// for barcodes, lot codes, expiration dates on packaging, low-cost cosmetic
// labelling.  GEOMETRICALLY a NO-OP — the workpiece shape is unchanged; we
// register the operation as a FeatureSignature whose pattern records the
// text, colour, font size, and target face for downstream tooling and
// inspection systems.
//
// Signature pattern:
//   - pattern.kind = "inkjet_mark"
//   - text, ink_color, font_size_mm, target_face_id, geometry_changed = false
//
// DFM checks:
//   DFM-INKJET-FONT : font_size_mm ≥ 0.3 mm (inkjet droplet resolution)
//   DFM-INPUT       : text non-empty, ink_color non-empty
//
// Recognize:
//   Metadata-only (ink is invisible to B-rep) — replay signature history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace inkjet_mark {

constexpr const char* kSkillId = "inkjet_mark";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    std::string text          = "";
    double      font_size_mm  = 2.0;
    std::string ink_color     = "black";   // "black" | "white" | "red" | ...
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace inkjet_mark
}  // namespace koocadcam::skill
