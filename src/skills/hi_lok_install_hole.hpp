#pragma once
// @lat: [[engine/skills#hi_lok_install_hole]]
//
// hi_lok_install_hole — COMPOUND aerospace Hi-Lok / Hi-Tigue / Hi-Lite
// fastener installation hole (HL-series, NAS1769).  Hi-Lok is a Huck-Lockbolt
// derivative used widely in commercial aviation: a hex pin engages a threaded
// collar with a break-off wrench drive.
//
//   sub-feature 1: primary drill at fastener_dia (close-fit clearance)
//   sub-feature 2: OPTIONAL countersink at 100° (NAS1097-style, normal) or
//                  130° (Hi-Lok flush-shear head NAS5238) — pick "none" to
//                  produce a protruding-head install hole only.
//
// Sequential pr::cut(wp, cutter) loop (slice-14 guard).
//
// Signature:
//   pattern.kind                   = "hi_lok_install_hole"
//   pattern.is_compound            = true (when countersink_style != "none")
//   pattern.aerospace_feature_type = "hi_lok_install"
//   pattern.fastener_dia           = fastener_dia_mm
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   DFM-INPUT       : all dimensions > 0
//   DFM-FAST-DIA    : fastener_dia ∈ {4.0, 4.8, 5.6, 6.4} mm
//   DFM-CSK-STYLE   : style ∈ {"100", "130", "none"}

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hi_lok_install_hole {

constexpr const char* kSkillId = "hi_lok_install_hole";

struct Input
{
    int         face_id           = -1;
    double      center_x_mm       = 0.0;
    double      center_y_mm       = 0.0;
    double      fastener_dia_mm   = 4.8;     // 4.0 / 4.8 / 5.6 / 6.4
    std::string countersink_style = "100";   // "100" | "130" | "none"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hi_lok_install_hole
}  // namespace koocadcam::skill
