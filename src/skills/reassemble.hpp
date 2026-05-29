#pragma once
// @lat: [[engine/skills#reassemble]]
//
// reassemble — reassembly of an assembly after maintenance.  Inverse of
// tear_down_inspect.  The operator supplies the reassembly order (a list of
// part identifiers in the order they were installed) and the number of
// torque-specified fasteners that were torqued during reassembly.
//
//        ┌──── part_A                              ┌──────────────┐
//        ├──── part_B   reassemble                 │  assembly    │
//        ├──── part_C   ──────────────────▶        │ (re-built)   │
//        └──── ...      (reassembly_order,         │              │
//                       torque_specs_count)        └──────────────┘
//          shape identical at slice-1 granularity;  history metadata stamped
//
// Signature pattern:
//   - pattern.kind                = "reassemble"
//   - pattern.reassembly_order    = [ ... ]
//   - pattern.part_count          = reassembly_order.size()
//   - pattern.torque_specs_count  = <int>
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   DFM-INPUT       reassembly_order empty            (error)
//   DFM-INPUT       torque_specs_count < 0            (error)
//   DFM-RA-DUP      duplicated id in reassembly_order (info — possible
//                   operator log mistake)
//   DFM-RA-TORQUE   torque_specs_count > part_count   (info — more torqued
//                   fasteners than parts is suspicious; verify spec)
//
// Synthesis:
//   NO geometric change.  Clones the workpiece and stamps the signature.
//
// Recognize:
//   Metadata-only replay.  Cannot recover reassembly history from B-rep.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace reassemble {

constexpr const char* kSkillId = "reassemble";

struct Input
{
    std::vector<std::string>  reassembly_order;
    int                       torque_specs_count = 0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace reassemble
}  // namespace koocadcam::skill
