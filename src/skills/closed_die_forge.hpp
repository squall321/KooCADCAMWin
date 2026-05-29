#pragma once
// @lat: [[engine/skills#closed_die_forge]]
//
// closed_die_forge — closed-die / impression-die forging.  Hot stock is
// squeezed inside a fully enclosed die cavity that imparts the final shape;
// excess material escapes as flash around the parting line and is later
// trimmed.  Because the die cavity defines the final geometry, slice-1
// models this as a **metadata-only** transformation: the workpiece comes
// out already shaped as the cavity dictates and we simply stamp a signature
// recording which die was used plus the flash thickness.
//
//        ┌──────────┐  upper die                ┌──────────┐
//   ───► │  stock   │  ──────────────────────►  │  shaped  │  + flash ──►
//        └──────────┘  lower die                └──────────┘
//                     impression die cavity     final part (geometry per shape_id)
//
// Synthesis strategy:
//   No geometric edit — the input workpiece is presumed to ALREADY be the
//   forged shape (typically supplied by a CAD model of the die cavity).  We
//   clone the workpiece, copy prior features, and stamp the signature.
//   Useful primarily for process-planning bookkeeping.
//
// Signature pattern:
//   - kind             = "closed_die_forge"
//   - shape_id          (string identifier of the die / cavity geometry)
//   - flash_thickness_mm (mm of flash trim required)
//   - geometry_changed  = false (we don't re-cut here)
//
// DFM checks:
//   DFM-INPUT        : shape_id empty; flash_thickness_mm < 0 — error
//   DFM-CDF-FLASH    : flash_thickness_mm > 5 mm — warning (excess flash)
//   DFM-CDF-FLASH    : flash_thickness_mm < 0.5 mm — info (tight tolerance)
//
// Recognize:
//   Geometry cannot reveal die identity → metadata-replay only.  Returns
//   the prior signature with confidence 1.0; nothing otherwise.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace closed_die_forge {

constexpr const char* kSkillId = "closed_die_forge";

struct Input
{
    std::string  shape_id;                  // die identifier (e.g. "watch_caseback_v3")
    double       flash_thickness_mm = 1.0;  // flash ring thickness (later trimmed)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace closed_die_forge
}  // namespace koocadcam::skill
