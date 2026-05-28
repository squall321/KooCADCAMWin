#pragma once
// @lat: [[engine/skills#anti_wear_treatment]]
//
// anti_wear_treatment — generic anti-wear surface treatment family.  Covers
// techniques that improve a face's tribological response (wear resistance,
// scuff resistance, lower friction) without changing macroscopic geometry:
//
//   - "laser_hardening"     — surface martensite via laser scan
//   - "induction_hardening" — case-harden a localised region (gear teeth)
//   - "surface_texturing"   — micro-dimple LIPSS / laser texturing for lubrication
//   - "ion_implantation"    — nitrogen / carbon ion bombardment hardening
//
// Used for skills that don't fit a more specific case-hardening contract
// but still need a record of the surface change.
//
//        ┌─────────────────────┐
//        │ ░░░░░░░░░░░░░░░░░░░ │  ← wear-resistant surface layer (depth_um deep)
//        │   (unchanged bulk)  │
//        └─────────────────────┘
//
// Signature pattern:
//   - pattern.kind                  = "anti_wear_treatment"
//   - pattern.target_face_id        = face treated
//   - pattern.technique             = "laser_hardening" | "induction_hardening" | …
//   - pattern.depth_um              = treatment depth (μm)
//   - pattern.hardness_increase_pct = expected hardness gain (%)
//   - pattern.geometry_changed      = false
//
// DFM checks:
//   DFM-INPUT                       depth_um ≤ 0 / hardness_increase_pct ≤ 0 → error
//   DFM-ANTIWEAR-DEPTH              depth_um ∉ [1.0, 2000.0]                  → error
//   DFM-ANTIWEAR-HARDNESS           hardness_increase_pct ∉ [5, 500]          → error
//   DFM-ANTIWEAR-TECHNIQUE          technique outside known set               → info
//
// Synthesis:
//   NO geometric change.  Clones the workpiece carrying a new signature.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace anti_wear_treatment {

constexpr const char* kSkillId = "anti_wear_treatment";

struct Input
{
    FaceDatum    entry_face;
    std::string  technique             = "laser_hardening";
    double       depth_um              = 100.0;   // typical 1 – 2000 μm
    double       hardness_increase_pct = 50.0;    // typical 5 – 500 %
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace anti_wear_treatment
}  // namespace koocadcam::skill
