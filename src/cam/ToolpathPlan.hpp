#pragma once
// @lat: [[engine/cam-3axis-verify]]
//
// CAM integration glue — the entry point that wires the (previously unused)
// toolpath generator + collision checker into one call a caller can use.
//
// A workpiece carries the FeatureSignatures of the skills applied to build it
// (Workpiece::features()).  For a reverse-engineered design replayed through the
// Executor, those are machining features (drill_hole, mill_circular_pocket,
// ...), which generateAllToolpaths turns into toolpaths.  planAndVerify
// generates every toolpath and collision-checks each against the workpiece,
// returning a single report — so RE -> execute -> verified toolpaths is one
// step instead of dead, separately-callable pieces.
//
// (Product-builder outputs carry product-step features, not machining skills, so
// they yield no toolpaths under the slice-1 generator — that is expected; this
// path is for the machining/RE side.)

#include "CollisionCheck.hpp"
#include "Toolpath.hpp"

#include "skills/Workpiece.hpp"

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace koocadcam::cam {

struct ToolpathReport
{
    std::vector<Toolpath>       toolpaths;     // one per machining feature recognised
    std::vector<CollisionEvent> collisions;    // all events across all toolpaths
    bool                        ok = true;      // true iff no collisions
};

// Generate toolpaths for every machining feature on `wp` and collision-check
// each against `wp`.  Uses the exact narrow-phase checker by default.
inline ToolpathReport planAndVerify(const skill::Workpiece& wp,
                                    double sample_step_mm = 0.5,
                                    double safe_z_margin  = 0.1)
{
    ToolpathReport report;
    report.toolpaths = generateAllToolpaths(wp.features());
    for (const auto& tp : report.toolpaths) {
        const auto events = checkPath(tp, wp, sample_step_mm, safe_z_margin);
        report.collisions.insert(report.collisions.end(), events.begin(), events.end());
    }
    report.ok = report.collisions.empty();
    return report;
}

// Emit one runnable G-code PROGRAM for a whole report: a single %/header, every
// toolpath's blocks (with a (TOOLCHANGE ...) comment on tool changes), then ONE
// program-end (M30, %).  Delegates to cam::toGCodeProgram so the program is
// terminated exactly once — concatenating per-toolpath toGCode() would emit an
// M30 after each feature and halt the machine at the first one.  This is the
// deliverable a user exports after a recognize -> execute -> verify pass.
inline std::string toGCodeProgram(const ToolpathReport& report)
{
    return toGCodeProgram(report.toolpaths, report.ok,
                          static_cast<int>(report.collisions.size()));
}

}  // namespace koocadcam::cam
