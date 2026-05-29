#pragma once
// @lat: [[engine/skills#knitting]]
//
// knitting — form fabric by interlooping a single (or a small set of) yarn
// rather than interlacing two yarn systems.  Unlike weaving, knitting can
// produce shaped 3D pieces directly (fully-fashioned shoe upper, glove,
// medical compression sleeve) without secondary cutting.  This skill is
// metadata-only at the OCCT level: the workpiece shape stays the yarn
// package volume; the signature records the loop topology.
//
//                ╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱     stitch_pattern = "jersey"
//                 │ │ │ │ │ │ │ │     gauge = stitches per inch
//                 │ │ │ │ │ │ │ │     yarn_type material descriptor
//                ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲
//
// Signature pattern:
//   - pattern.kind             = "knitting"
//   - pattern.gauge            = stitches per inch (needle pitch)
//   - pattern.yarn_type        = "cotton" | "wool" | "polyester" | "nylon" | "blend"
//   - pattern.stitch_pattern   = "jersey" | "rib" | "interlock" | "purl" | "jacquard"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   gauge > 0                       (DFM-INPUT          error)
//   yarn_type recognised            (DFM-TEX-YARN       info)
//   stitch_pattern recognised       (DFM-TEX-STITCH     info)
//   gauge typical range [4, 60]     (DFM-TEX-GAUGE      info)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace knitting {

constexpr const char* kSkillId = "knitting";

struct Input
{
    double       gauge          = 14.0;        // stitches per inch (needle pitch)
    std::string  yarn_type      = "cotton";    // cotton | wool | polyester | nylon | blend
    std::string  stitch_pattern = "jersey";    // jersey | rib | interlock | purl | jacquard
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace knitting
}  // namespace koocadcam::skill
