#pragma once
// @lat: [[engine/skills#offset_print]]
//
// offset_print — lithographic offset printing.  Image is transferred from
// plate to rubber blanket then to paper.  Macroscopic B-rep unchanged;
// printing metadata is stamped onto the workpiece.
//
// Signature pattern:
//   pattern.kind             = "offset_print"
//   pattern.paper_gsm        = double (60..400 typical)
//   pattern.ink_set          = int (CMYK channel count 1..6+)
//   pattern.sheets           = int
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             paper_gsm ≤ 0 or sheets ≤ 0 or ink_set < 1 (error)
//   DFM-PAPER-LIGHT       paper_gsm < 60 (info — risk of show-through)
//   DFM-PAPER-HEAVY       paper_gsm > 400 (info — board, may need special press)
//   DFM-INK-SET           ink_set > 6 (info — beyond hexachrome / specialty)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace offset_print {

constexpr const char* kSkillId = "offset_print";

struct Input
{
    double paper_gsm = 120.0;   // grams per square meter
    int    ink_set   = 4;       // CMYK = 4
    int    sheets    = 1000;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace offset_print
}  // namespace koocadcam::skill
