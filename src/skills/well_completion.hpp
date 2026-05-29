#pragma once
// @lat: [[engine/skills#well_completion]]
//
// well_completion — oil and gas downhole well completion record.  Captures
// the post-drill completion configuration of a hydrocarbon production well:
// production casing string set to the producing zone, perforations shot
// through the casing/cement into the formation, and the per-well identity
// used to tie completion-quality data back to a corporate well registry.
// This is a process-planning record only — the macroscopic workpiece (the
// surface or downhole module model under construction) is NOT modified.
//
//        ┌──────────┐  surface
//        │          │
//        │   ┌──┐   │   ← well_id stamp
//        │   │  │   │
//        │   │  │   │   ← casing_dia_mm  (production casing OD)
//        │   │  │   │
//        │   │░░│   │   ← perforation_count rounds shot through casing
//        │   │░░│   │     into formation @ depth_m
//        │   └──┘   │
//        └──────────┘
//
// Signature pattern:
//   - pattern.kind              = "well_completion"
//   - pattern.well_id           = string (e.g. "K-15-A1")
//   - pattern.depth_m
//   - pattern.casing_dia_mm
//   - pattern.perforation_count
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   well_id non-empty                            (DFM-INPUT error)
//   depth_m > 0                                  (DFM-INPUT error)
//   casing_dia_mm > 0                            (DFM-INPUT error)
//   casing_dia_mm ∈ [114.3, 339.7] mm typical    (DFM-WC-CASING info)
//   perforation_count > 0                        (DFM-INPUT error)
//   perforation_count > 200                      (DFM-WC-PERF info — heavy shot density)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — completion is a downhole record, not a B-rep feature.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace well_completion {

constexpr const char* kSkillId = "well_completion";

struct Input
{
    std::string  well_id           = "WELL-001";
    double       depth_m           = 2500.0;
    double       casing_dia_mm     = 177.8;       // 7" production casing typical
    int          perforation_count = 24;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace well_completion
}  // namespace koocadcam::skill
