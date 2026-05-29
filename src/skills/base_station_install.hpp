#pragma once
// @lat: [[engine/skills#base_station_install]]
//
// base_station_install — commissioning record for a cellular / radio base
// station: site identifier, total equipment power draw, and the number of
// active sectors served (typical macro cell has 3 sectors at 120° each).
// The workpiece B-rep is unchanged — only the site-frame install record
// is stamped.
//
//                ─── sector 0 (0°)
//                  ╲
//                   ╲  ╔══════════╗
//                    ╲ ║   BBU /  ║      sector 1 (120°)
//                     ╳║   eNodeB ║──────
//                    ╱ ║          ║
//                   ╱  ╚══════════╝
//                  ╱       ↑
//                ─── sector 2 (240°)
//                       (equipment_kw total)
//
// Signature pattern:
//   - pattern.kind             = "base_station_install"
//   - pattern.site_id          = string
//   - pattern.equipment_kw
//   - pattern.sectors_count    = int (typically 1, 3, or 6)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   site_id non-empty                          (DFM-INPUT error)
//   equipment_kw > 0                           (DFM-INPUT error)
//   equipment_kw > 50                          (DFM-BS-POWER warning — large macro)
//   sectors_count ≥ 1                          (DFM-INPUT error)
//   sectors_count ∉ {1, 3, 6}                  (DFM-BS-SECTORS info — atypical)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace base_station_install {

constexpr const char* kSkillId = "base_station_install";

struct Input
{
    std::string  site_id        = "BTS-001";
    double       equipment_kw   = 5.0;
    int          sectors_count  = 3;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace base_station_install
}  // namespace koocadcam::skill
