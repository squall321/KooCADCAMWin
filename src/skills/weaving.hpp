#pragma once
// @lat: [[engine/skills#weaving]]
//
// weaving — interlace warp (longitudinal) and weft (transverse) yarns on a
// loom to produce a flat fabric web.  The single-workpiece OCCT shape that
// the skill ingests is the raw greige yarn package; the produced fabric is
// represented topologically by the same shape with an attached
// FeatureSignature.  This is a metadata-only step: macroscopic boundary
// representation is unchanged, the material microstructure (yarn → fabric)
// transitions in the signature only.
//
//           warp  ──────┐
//           warp  ──────┤
//           warp  ──────┤   ◄── shedding + pick + beat-up ──►   fabric web
//           warp  ──────┤                                       (W × interlace)
//           warp  ──────┘
//                    ↕     weft (filling) crosses each pick
//
// Signature pattern:
//   - pattern.kind             = "weaving"
//   - pattern.loom_type        = "rapier" | "air_jet" | "water_jet" | "shuttle" | "projectile"
//   - pattern.warp_count       = ends per cm
//   - pattern.weft_count       = picks per cm
//   - pattern.weave_pattern    = "plain" | "twill" | "satin" | "basket" | "leno"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   loom_type recognised                       (DFM-TEX-LOOM       info)
//   weave_pattern recognised                   (DFM-TEX-WEAVE      info)
//   warp_count > 0                             (DFM-INPUT          error)
//   weft_count > 0                             (DFM-INPUT          error)
//   weft_count / warp_count ratio sane         (DFM-TEX-BALANCE    info)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata-only — interlace topology is sub-OCCT.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace weaving {

constexpr const char* kSkillId = "weaving";

struct Input
{
    std::string  loom_type     = "rapier";         // rapier | air_jet | water_jet | shuttle | projectile
    double       warp_count    = 28.0;             // ends per cm
    double       weft_count    = 24.0;             // picks per cm
    std::string  weave_pattern = "plain";          // plain | twill | satin | basket | leno
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace weaving
}  // namespace koocadcam::skill
