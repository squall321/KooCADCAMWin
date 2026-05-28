#pragma once
// @lat: [[engine/skills#abrasive_flow]]
//
// abrasive_flow — Abrasive Flow Machining (AFM).
//
// A viscoelastic putty loaded with abrasive grit is forced back-and-forth
// through an INTERNAL CHANNEL of the workpiece under pressure.  The
// flowing medium polishes / deburrs the channel walls without touching
// external surfaces.  Used for:
//   - automotive fuel injector nozzle blending
//   - hydraulic manifold deburr
//   - extrusion die polishing
//   - turbine blade cooling channel finish
//
// AFM removes only microns of material — too small to model as a Boolean
// cut.  Like the heat-treatment skills, this is a METADATA-ONLY operation:
// the workpiece SHAPE is unchanged; surface roughness Ra and edge condition
// improve.  We record the BEFORE/AFTER roughness in the FeatureSignature so
// downstream consumers can verify the polish step.
//
// Geometry: identical input and output workpiece (no Boolean op).
//
// Signature pattern:
//   - kind                = "abrasive_flow"
//   - channel_id          (datum reference for the polished channel)
//   - ra_before_um        (input)
//   - ra_after_um         (input, must be < ra_before_um)
//   - cycles              (number of putty extrusion strokes)
//   - geometry_changed    = false
//
// DFM checks:
//   DFM-INPUT       cycles ≤ 0, ra_before ≤ 0
//   DFM-AFM-RA      ra_after >= ra_before → error (no improvement)
//   DFM-AFM-RA-FLOOR  ra_after < 0.05 μm → warning (below AFM practical floor)
//   DFM-AFM-CYCLES  cycles > 200 → info (diminishing returns)
//
// Recognize:
//   Pattern follows `anneal.cpp` — no geometric trace.  recognize() returns
//   only metadata-replay candidates from wp.features().

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace abrasive_flow {

constexpr const char* kSkillId = "abrasive_flow";

struct Input
{
    // Datum hint — descriptive only (AFM polishes "the internal channel");
    // the skill does not resolve geometry, but recording the hint lets a
    // downstream planner replay the operation against a re-imported part.
    std::string  channel_label   = "main_channel";
    double       ra_before_um    = 1.6;    // typical milled surface
    double       ra_after_um     = 0.4;    // AFM textbook delivery
    int          cycles          = 50;
    std::string  abrasive_media  = "SiC_400_grit";
    double       pressure_mpa    = 7.0;    // typ. 5-10 MPa
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace abrasive_flow
}  // namespace koocadcam::skill
