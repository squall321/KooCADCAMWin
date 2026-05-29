#pragma once
// @lat: [[engine/skills#automated_storage]]
//
// automated_storage — AS/RS (Automated Storage and Retrieval System) bin
// transfer.  A crane- or shuttle-driven AS/RS retrieves or stows a part
// into an addressable bin location at a given storage height.  This is a
// pure material-handling operation: the workpiece geometry is NOT
// modified; only the storage location is recorded so a downstream WMS /
// fleet controller can issue the corresponding put/pick command.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║   ┌──────┐  ╭───────────╮  ┌────────────┐              ║
//          ║   │ part │ ▶│   AS/RS   │ ▶│ bin slot   │              ║
//          ║   │ I/O  │  │  shuttle  │  │ A-12-Lv03  │              ║
//          ║   └──────┘  ╰───────────╯  └────────────┘              ║
//          ║                                                        ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind             = "automated_storage"
//   - pattern.bin_id           = string (e.g., "A-12-Lv03")
//   - pattern.storage_height_m
//   - pattern.load_class       = "light" | "medium" | "heavy"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   bin_id non-empty                            (DFM-INPUT error)
//   storage_height_m > 0                        (DFM-INPUT error)
//   storage_height_m ≤ 30                       (DFM-ASRS-HEIGHT info — typ rack)
//   load_class ∈ known set                      (DFM-ASRS-CLASS info)
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

namespace automated_storage {

constexpr const char* kSkillId = "automated_storage";

struct Input
{
    std::string  bin_id           = "A-01-Lv01";
    double       storage_height_m = 5.0;
    std::string  load_class       = "medium";   // "light" | "medium" | "heavy"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace automated_storage
}  // namespace koocadcam::skill
