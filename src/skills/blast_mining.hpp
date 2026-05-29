#pragma once
// @lat: [[engine/skills#blast_mining]]
//
// blast_mining — production blast on a mining bench.  A grid of pre-drilled
// blastholes is loaded with explosive (ANFO, emulsion, or watergel slurry)
// and detonated to fragment rock.  KooCADCAM does NOT simulate fragmentation
// or muck pile geometry; this skill captures the BLAST RECORD (hole pattern,
// charge weight, energy type) so that pit-design / sequencing can replay it.
//
//        ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░       ← bench top
//          │   │   │   │   │   │   │
//          ▼   ▼   ▼   ▼   ▼   ▼   ▼          ← pattern_holes blastholes
//          ▮   ▮   ▮   ▮   ▮   ▮   ▮          ← charge_per_hole_kg each
//
// Signature pattern:
//   pattern.kind                 = "blast_mining"
//   pattern.pattern_holes        = int
//   pattern.charge_per_hole_kg   = double
//   pattern.explosive_type       = "anfo" | "emulsion" | "watergel"
//   pattern.geometry_changed     = false
//
// DFM:
//   DFM-INPUT              pattern_holes ≤ 0 or charge_per_hole_kg ≤ 0
//   DFM-EXPLOSIVE-TYPE     explosive_type not in {anfo, emulsion, watergel}
//   DFM-CHARGE-HIGH        charge_per_hole_kg > 500 kg (info)
//   DFM-PATTERN-LARGE      pattern_holes > 500 (mega-blast advisory, info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace blast_mining {

constexpr const char* kSkillId = "blast_mining";

struct Input
{
    int         pattern_holes      = 48;     // typical production bench shot
    double      charge_per_hole_kg = 120.0;
    std::string explosive_type     = "anfo"; // "anfo" | "emulsion" | "watergel"
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace blast_mining
}  // namespace koocadcam::skill
