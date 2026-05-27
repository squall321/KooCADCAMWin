#pragma once
// @lat: [[engine/reverse-route#Recognizer]]
//
// Layer 4 — Reverse Engineering pipeline.
//
// Given a Workpiece (typically loaded from a STEP file or produced by an
// earlier process), runs every registered skill's recognize() to collect
// candidate features, then optionally deduplicates competing claims and
// assembles a plausible ProcessPlan that COULD have produced the workpiece.
//
// Pipeline:
//
//   analyze()            — run every recognizer, return all candidates
//                          sorted by confidence (descending).
//   analyzeFiltered()    — drop candidates below a confidence threshold.
//   dedupe()             — collapse competing claims on the same geometry
//                          (e.g. drill_hole + bore_cylindrical both claim
//                          a cylindrical face) into the single
//                          highest-confidence candidate per fingerprint.
//   inferProcessPlan()   — order candidates by machining-stage heuristics
//                          (stock removal → holes → edge ops) and emit a
//                          ProcessPlan whose steps replay the inferred
//                          synthesis history.
//
// Slice-1 ordering is purely category-based; topological sequencing (e.g.
// "drill the hole BEFORE the chamfer that touches its rim") is a future
// improvement.  Per-step `depends_on` is left empty in slice-1.

#include "process/ProcessPlan.hpp"
#include "skills/Skill.hpp"
#include "skills/Workpiece.hpp"

#include <functional>
#include <vector>

namespace koocadcam::re {

// Function-pointer alias for a skill's recognize() entry point.
using RecognizeFn =
    std::function<std::vector<skill::RecognizedFeature>(const skill::Workpiece&)>;

// Static recognizer registry — one entry per skill whose recognize() is
// reliable enough to participate in RE.  Built lazily on first call.
const std::vector<RecognizeFn>& recognizerRegistry();

// Run every registered skill's recognize() on the workpiece, collect every
// candidate from every skill, return them sorted by confidence (descending).
std::vector<skill::RecognizedFeature>
analyze(const skill::Workpiece& wp);

// Filter candidates below `min_confidence` (default 0.7).
std::vector<skill::RecognizedFeature>
analyzeFiltered(const skill::Workpiece& wp, double min_confidence = 0.7);

// De-duplicate competing claims.  Two recognizers may both fire on the same
// piece of geometry (e.g. drill_hole and bore_cylindrical both recognise a
// cylindrical face).  For each set of candidates whose matched_geometry
// overlaps, keep only the single highest-confidence candidate.
std::vector<skill::RecognizedFeature>
dedupe(const std::vector<skill::RecognizedFeature>& candidates);

// Build a ProcessPlan from the recognized features.  Steps are appended in
// the slice-1 order:
//   Group A (stock removal):  hollow_cavity, mill_*_pocket, mill_slot, …
//   Group B (holes / drills): drill_*, counterbore, bore_*, ream, …
//   Group C (edge ops):       chamfer_edge, fillet_edge, face_milling
// Within each group, higher-confidence candidates come first.
process::ProcessPlan
inferProcessPlan(const skill::Workpiece& wp, double min_confidence = 0.7);

}  // namespace koocadcam::re
