#pragma once
// @lat: [[engine/skills#crankshaft_journal_with_fillet]]
//
// crankshaft_journal_with_fillet — AUTOMOTIVE ICE engine compound: turn a
// main-bearing journal step on a crankshaft cylinder, then apply a stress
// relief fillet at BOTH ends of the journal step.  Fillet is critical to
// crankshaft fatigue life (SAE J recommendations: 2-4 mm radius typical).
//
// Sub-feature chain (2 conceptual ops, 3 visible features):
//   1. TURN/CUT — remove the annular ring outside journal_dia over
//      journal_length, centered at journal_position_z_mm.
//   2. FILLET — apply fillet_radius at both step edges (top + bottom).
//
// Signature:
//   pattern.kind                  = "crankshaft_journal_with_fillet"
//   pattern.engine_feature_type   = "crankshaft_journal"
//   pattern.subfeature_count      = 3  (cut + 2 fillets)
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - face_id refers to a cylindrical face (shaft)
//   - journal_dia_mm > 0 and strictly less than current shaft OD
//   - journal_length_mm > 0
//   - fillet_radius_mm in (0.5, 8) mm — fatigue spec sweet spot

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace crankshaft_journal_with_fillet {

constexpr const char* kSkillId = "crankshaft_journal_with_fillet";

struct Input
{
    int     face_id              = -1;
    double  journal_position_z_mm = 0.0;
    double  journal_dia_mm        = 50.0;
    double  journal_length_mm     = 20.0;
    double  fillet_radius_mm      = 3.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace crankshaft_journal_with_fillet
}  // namespace koocadcam::skill
