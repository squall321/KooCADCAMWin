#pragma once
// @lat: [[engine/skills#digital_print]]
//
// digital_print — short-run digital printing (toner xerography or inkjet),
// no plate, variable data per sheet.  Macroscopic B-rep unchanged; only
// metadata is stamped onto the workpiece.
//
// Signature pattern:
//   pattern.kind             = "digital_print"
//   pattern.dpi              = int (resolution; 300..2400 typical)
//   pattern.ink_type         = string ("toner" | "aqueous" | "uv" | "latex")
//   pattern.sheets           = int
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             dpi ≤ 0 or sheets ≤ 0 (error)
//   DFM-INPUT             ink_type empty or unknown (error)
//   DFM-DPI-LOW           dpi < 300 (info — draft quality)
//   DFM-DPI-HIGH          dpi > 2400 (info — specialty / longer cycle)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace digital_print {

constexpr const char* kSkillId = "digital_print";

struct Input
{
    int          dpi      = 1200;
    std::string  ink_type = "toner";   // "toner" | "aqueous" | "uv" | "latex"
    int          sheets   = 500;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace digital_print
}  // namespace koocadcam::skill
