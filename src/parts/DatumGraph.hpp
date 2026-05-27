#pragma once
// @lat: [[engine/skills#Layer 6 Datum-part dependency graph]]
//
// DatumGraph — directed graph linking ProcessPlan step indices to Part ids.
//
// This is the lynchpin of the auto-re-evaluation workflow:
//
//   1. User imports an existing frame + parts layout.
//   2. Layer 4 RE produces a ProcessPlan v1.
//   3. extractHeuristicDependencies(plan, layout) builds a DatumGraph
//      mapping each step → the part(s) whose AABB it sits on/in/near.
//   4. User swaps a part (e.g. PCB shifts 5 mm).  PartsLayout::diff is run
//      against the original layout.
//   5. DatumGraph::invalidatedSteps(diff) returns the step indices to re-
//      validate / re-edit; the Layer-5 PlanEditor proposes EditOps.
//
// The graph stores edges as step_index → list<part_id>.  The reverse map
// (part_id → list<step_index>) is computed lazily on demand by
// stepsDependentOn() — there is no eager reverse index, so adding an edge
// is O(1) but a part-anchored query is O(N) over all step edges.  For
// realistic plans (≤ 100 steps, ≤ 50 parts) this is irrelevant.
//
// JSON shape:
//   {
//     "version": 1,
//     "edges": [
//       { "step_index": 0, "part_ids": ["pcb_main"] },
//       { "step_index": 3, "part_ids": ["display", "battery"] },
//       ...
//     ]
//   }

#include "PartsLayout.hpp"

#if __has_include("process/ProcessPlan.hpp")
#  include "process/ProcessPlan.hpp"
#  define KOO_PARTS_HAS_REAL_PROCESS_PLAN 1
#else
#  define KOO_PARTS_HAS_REAL_PROCESS_PLAN 0
#endif

#include <nlohmann/json.hpp>

#include <map>
#include <string>
#include <vector>

namespace koocadcam::parts {

class DatumGraph
{
public:
    DatumGraph() = default;

    // ── Mutation ─────────────────────────────────────────────────────────

    // Record that the step at `step_index` depends on `part_id`.  Adding
    // the same edge twice is a no-op (the part_id is deduplicated within
    // the step's list).
    void addDependency(int step_index, const std::string& part_id);

    // ── Queries ──────────────────────────────────────────────────────────

    // Which steps depend on this part?  Sorted ascending by step_index.
    std::vector<int> stepsDependentOn(const std::string& part_id) const;

    // Which parts does this step depend on?  Returns a copy of the
    // step's part_id list (or an empty vector if the step has no edges).
    std::vector<std::string> partsForStep(int step_index) const;

    // Given a parts diff, return the (deduplicated, ascending) set of step
    // indices that are potentially invalidated.  A diff entry with kind
    // "added" cannot invalidate any pre-existing step (no edges yet) and
    // is silently ignored — but "removed", "moved", "resized" all map to
    // their respective dependent step indices.
    std::vector<int> invalidatedSteps(
        const std::vector<PartsLayout::PartDiff>& diff) const;

    // ── Serialisation ───────────────────────────────────────────────────

    nlohmann::json    toJson() const;
    static DatumGraph fromJson(const nlohmann::json& j);

    // Direct read access for tests / introspection.
    const std::map<int, std::vector<std::string>>& edges() const { return m_stepToParts; }

private:
    // step_index → list of part_ids (deduplicated, insertion-ordered).
    std::map<int, std::vector<std::string>> m_stepToParts;
};

// ── Heuristic extractor ─────────────────────────────────────────────────
//
// Scan a ProcessPlan's step params for fields named like position_x_mm,
// center_x_mm, offset_x_mm (and the y/z analogues).  For each step that
// yields a usable (x, y, z) — with missing components treated as 0.0 — find
// the part whose AABB centre is closest in Euclidean distance.  If the
// nearest part is within `match_tolerance_mm` (default 5.0 mm), record an
// edge step → part.
//
// What it CATCHES:
//   - drill_hole / counterbore / countersink / mill_*_pocket / mill_slot /
//     bore_* / engrave_text (skills whose params encode a centre point).
//
// What it MISSES (by design — full geometric analysis is future work):
//   - face_milling / fillet_edge / chamfer_edge (no position field — these
//     reference faces/edges, not (x,y) coords).
//   - profile_milling (path-based — depends on the WHOLE part, not a point).
//   - Multi-part dependencies (e.g. a slot that straddles the PCB AND the
//     battery).  Only the nearest part is recorded.
//   - Skills where the position field overlaps the AABB of an UNRELATED
//     part (false positive risk — the tolerance gating keeps this bounded
//     but not zero).
//
// `plan` is read by const-ref via the StepInvocation::params JSON only —
// the heuristic does NOT need the full Layer-3 build graph, just the JSON
// representation.  Therefore the function works equally with the real
// koo_process ProcessPlan and with any other class exposing
// `steps()` -> `range of { skill_id, params }`.
#if KOO_PARTS_HAS_REAL_PROCESS_PLAN
DatumGraph extractHeuristicDependencies(
    const process::ProcessPlan& plan,
    const PartsLayout&          layout,
    double                      match_tolerance_mm = 5.0);
#endif

// Plan-agnostic overload: take a vector<json> of step params (one per step)
// directly.  Convenient for unit tests and for callers that have not
// (yet) linked koo_process.  The first overload above is a thin wrapper
// around this one.
DatumGraph extractHeuristicDependencies(
    const std::vector<nlohmann::json>& stepParams,
    const PartsLayout&                 layout,
    double                             match_tolerance_mm = 5.0);

}  // namespace koocadcam::parts
