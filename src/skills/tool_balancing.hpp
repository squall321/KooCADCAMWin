#pragma once
// @lat: [[engine/skills#tool_balancing]]
//
// tool_balancing — high-speed cutting-tool assembly (tool + holder) is
// dynamically balanced to a target ISO 1940/1 grade.  Distinct from
// rotor_balancing (which handles turbine / compressor rotors): this is
// for HSK / shrink-fit / hydraulic milling holders running 20k-60k rpm.
// Unbalance at spindle speed causes runout, surface chatter, and reduced
// bearing life — hence the per-tool balancing step.
//
//                    ↻ ω
//              ╔═══════════╗   tool_balancing
//              ║   tool +  ║  ────────────────▶  achieved residual U
//              ║   holder  ║  add weight or       ≤ grade_g_mm_kg · m
//              ║           ║  drill flute back
//              ╚═══════════╝
//
// Signature pattern:
//   - pattern.kind            = "tool_balancing"
//   - pattern.tool_id         = <input>
//   - pattern.grade_g_mm_kg   = <input>      e.g. 1.0 for G1.0 mm/kg
//   - pattern.target_rpm      = <input>
//   - pattern.geometry_changed= false
//
// DFM checks:
//   tool_id non-empty                    (DFM-INPUT error if not)
//   grade_g_mm_kg > 0                    (DFM-INPUT error if not)
//   target_rpm > 0                       (DFM-INPUT error if not)
//   grade_g_mm_kg in [0.4, 16]           (DFM-TB-GRADE info if outside)
//   target_rpm in [3000, 100000]         (DFM-TB-RPM info if outside)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
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

namespace tool_balancing {

constexpr const char* kSkillId = "tool_balancing";

struct Input
{
    std::string tool_id        = "EM-12MM-HSK63-001";
    double      grade_g_mm_kg  = 1.0;       // ISO 1940 G value in g·mm/kg
    double      target_rpm     = 24000.0;   // typical HSC milling
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tool_balancing
}  // namespace koocadcam::skill
