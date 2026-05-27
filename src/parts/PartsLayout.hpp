#pragma once
// @lat: [[engine/skills#Layer 6 Parts layout]]
//
// PartsLayout — collection of internal Parts with diff + JSON I/O.
//
// The PartsLayout is the "source of truth" for the *physical* positions of
// every internal part in the assembly.  It is consumed by:
//
//   - DatumGraph::extractHeuristicDependencies — to seed the
//     step ↔ part dependency edges.
//   - DatumGraph::invalidatedSteps — by computing diff(old, new) and
//     mapping affected part_ids onto step indices.
//
// Two layouts are compared via `diff(other)`.  Each PartDiff captures one
// of four kinds of change:
//
//   - "added"    — part exists in `other`, not in `this`
//   - "removed"  — part exists in `this`, not in `other`
//   - "moved"    — same id, AABB centre differs by > kMovedTolMm (1e-3)
//   - "resized"  — same id, AABB extent (any axis) differs by > kSizeTolMm
//
// The details JSON for each kind carries the *delta* (from→to centres, or
// from→to size) so downstream consumers can decide whether the magnitude
// warrants re-running a particular step.
//
// JSON serialisation
// ──────────────────
// The OCCT TopoDS_Shape is NOT embedded in JSON — only the step_path is
// recorded.  Reconstructing the geometry is the caller's responsibility
// (call `fromStepFiles` again, or hydrate via the placement-only path).
//
// JSON shape:
//   {
//     "version": 1,
//     "parts": [
//       { "id": "...", "step_path": "...",
//         "aabb": { "xMin": .., "yMin": .., "zMin": ..,
//                   "xMax": .., "yMax": .., "zMax": .. },
//         "placement": [16 doubles, row-major 4x4] },
//       ...
//     ]
//   }

#include "Part.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace koocadcam::parts {

class PartsLayout
{
public:
    PartsLayout() = default;

    // Construct from a list of STEP files.  Each STEP file = one part with
    // the file stem as its id.  Files that fail to load are SKIPPED (a
    // warning is logged); the resulting layout contains only successfully
    // loaded parts.  Order in the returned layout matches the input order
    // of successful loads.
    static PartsLayout fromStepFiles(const std::vector<std::filesystem::path>& steps);

    // Direct add (primarily for tests + programmatic construction).  The
    // AABB fields are filled from the Part's existing fields if non-zero;
    // otherwise, if `part.shape` is non-null, BRepBndLib is used to compute
    // them.  When the shape is null and the AABB is zero, the part is
    // recorded as a point at the origin (useful for tests that only need
    // the id+placement to drive the dependency graph).
    void addPart(Part part);

    const std::vector<Part>& parts() const { return m_parts; }
    const Part*              findById(const std::string& id) const;

    // ── Diff ─────────────────────────────────────────────────────────────

    struct PartDiff
    {
        std::string    part_id;
        std::string    kind;     // "added" | "removed" | "moved" | "resized"
        nlohmann::json details;  // delta description (see top of header)
    };

    // Compute the diff between *this* and `other`.
    //
    // Semantics:
    //   - this is treated as the OLD layout, other as the NEW layout.
    //   - "added"   parts are in other but not in this.
    //   - "removed" parts are in this but not in other.
    //   - For parts in both, "moved" or "resized" can be emitted.  Both can
    //     appear for the same part if the part both translated *and*
    //     changed dimension.
    std::vector<PartDiff> diff(const PartsLayout& other) const;

    // ── JSON I/O ────────────────────────────────────────────────────────

    nlohmann::json     toJson() const;
    static PartsLayout fromJson(const nlohmann::json& j);

    // Tolerances used by diff().  Public so tests can poke at them.
    static constexpr double kMovedTolMm = 1.0e-3;
    static constexpr double kSizeTolMm  = 1.0e-3;

private:
    std::vector<Part> m_parts;
};

}  // namespace koocadcam::parts
