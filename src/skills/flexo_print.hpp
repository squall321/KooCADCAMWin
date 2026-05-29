#pragma once
// @lat: [[engine/skills#flexo_print]]
//
// flexo_print — flexographic printing.  Raised-image rubber/photopolymer
// plate inks from an anilox cell roller; common for corrugated, labels,
// flexible packaging.  Macroscopic B-rep unchanged; metadata stamped.
//
// Signature pattern:
//   pattern.kind             = "flexo_print"
//   pattern.anilox_lpi       = int (cells per linear inch — 200..1400)
//   pattern.dot_size_um      = double (min printable dot — 20..120)
//   pattern.ink_type         = string ("water" | "solvent" | "uv")
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             anilox_lpi ≤ 0 or dot_size_um ≤ 0 (error)
//   DFM-INPUT             ink_type empty or unknown (error)
//   DFM-ANILOX-COARSE     anilox_lpi < 200 (info — heavy lay-down)
//   DFM-ANILOX-FINE       anilox_lpi > 1400 (info — process colour highlights)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace flexo_print {

constexpr const char* kSkillId = "flexo_print";

struct Input
{
    int          anilox_lpi  = 800;       // cells per linear inch
    double       dot_size_um = 40.0;
    std::string  ink_type    = "water";   // "water" | "solvent" | "uv"
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace flexo_print
}  // namespace koocadcam::skill
