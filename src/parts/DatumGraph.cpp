// @lat: [[engine/skills#Layer 6 Datum-part dependency graph]]

#include "DatumGraph.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <array>
#include <map>
#include <optional>
#include <set>

namespace koocadcam::parts {

using nlohmann::json;

// ── DatumGraph members ───────────────────────────────────────────────────

void DatumGraph::addDependency(int step_index, const std::string& part_id)
{
    auto& list = m_stepToParts[step_index];
    if (std::find(list.begin(), list.end(), part_id) == list.end()) {
        list.push_back(part_id);
    }
}

std::vector<int> DatumGraph::stepsDependentOn(const std::string& part_id) const
{
    std::vector<int> out;
    for (const auto& [step, parts] : m_stepToParts) {
        if (std::find(parts.begin(), parts.end(), part_id) != parts.end()) {
            out.push_back(step);
        }
    }
    // m_stepToParts is std::map, so insertion order above is already sorted
    // ascending.  No further sort needed.
    return out;
}

std::vector<std::string> DatumGraph::partsForStep(int step_index) const
{
    auto it = m_stepToParts.find(step_index);
    if (it == m_stepToParts.end()) return {};
    return it->second;
}

std::vector<int>
DatumGraph::invalidatedSteps(const std::vector<PartsLayout::PartDiff>& diff) const
{
    std::set<int> hit;
    for (const auto& d : diff) {
        // "added" parts have no edges yet — they can't invalidate steps.
        if (d.kind == "added") continue;

        for (int step : stepsDependentOn(d.part_id)) {
            hit.insert(step);
        }
    }
    return { hit.begin(), hit.end() };  // ascending by set semantics
}

// ── Serialisation ────────────────────────────────────────────────────────

json DatumGraph::toJson() const
{
    json out = {
        { "version", 1 },
        { "edges",   json::array() },
    };
    for (const auto& [step, parts] : m_stepToParts) {
        out["edges"].push_back({
            { "step_index", step },
            { "part_ids",   parts },
        });
    }
    return out;
}

DatumGraph DatumGraph::fromJson(const json& j)
{
    DatumGraph g;
    if (!j.contains("edges") || !j["edges"].is_array()) {
        spdlog::warn("DatumGraph::fromJson: missing or non-array 'edges'");
        return g;
    }
    for (const auto& e : j["edges"]) {
        if (!e.contains("step_index") || !e["step_index"].is_number_integer()) continue;
        const int idx = e["step_index"].get<int>();
        if (!e.contains("part_ids") || !e["part_ids"].is_array()) continue;
        for (const auto& pid : e["part_ids"]) {
            if (pid.is_string()) g.addDependency(idx, pid.get<std::string>());
        }
    }
    return g;
}

// ── Heuristic extractor ──────────────────────────────────────────────────

namespace {

// Try to extract a coordinate field from a step's params JSON.  Recognised
// suffixes are *_x_mm, *_y_mm, *_z_mm.  Recognised prefixes are
// "position_", "center_", "offset_".  Returns nullopt if no such field
// exists (e.g. for face_milling whose params encode a face datum only).
std::optional<double> extractAxisCoord(const json& params, char axis)
{
    if (!params.is_object()) return std::nullopt;
    static const std::vector<std::string> kPrefixes = {
        "position_", "center_", "offset_"
    };
    for (const auto& pre : kPrefixes) {
        const std::string key = pre + axis + "_mm";
        if (params.contains(key) && params[key].is_number()) {
            return params[key].get<double>();
        }
    }
    return std::nullopt;
}

// Add `delta` to whichever coordinate field exists for `axis` (the same
// prefix set extractAxisCoord scans).  Shifts only the first matching field
// per axis; no-op if the step has no coordinate for that axis.
void shiftAxisCoord(json& params, char axis, double delta)
{
    if (!params.is_object() || delta == 0.0) return;
    static const std::vector<std::string> kPrefixes = {
        "position_", "center_", "offset_"
    };
    for (const auto& pre : kPrefixes) {
        const std::string key = pre + axis + "_mm";
        if (params.contains(key) && params[key].is_number()) {
            params[key] = params[key].get<double>() + delta;
            return;
        }
    }
}

// Read the step's (x, y, z).  Returns nullopt if NO axis is found at all
// (so the step is "position-less" — not amenable to point matching).  If
// some axes are missing but others are present, the missing ones default
// to 0.0 (e.g. a 2D pocket on the top face encodes only x + y).
struct StepPoint { double x, y, z; bool any; };
StepPoint pointFromParams(const json& params)
{
    StepPoint pt{0.0, 0.0, 0.0, false};
    auto vx = extractAxisCoord(params, 'x');
    auto vy = extractAxisCoord(params, 'y');
    auto vz = extractAxisCoord(params, 'z');
    if (vx) { pt.x = *vx; pt.any = true; }
    if (vy) { pt.y = *vy; pt.any = true; }
    if (vz) { pt.z = *vz; pt.any = true; }
    return pt;
}

double dist2(double ax, double ay, double az,
             double bx, double by, double bz)
{
    const double dx = bx - ax, dy = by - ay, dz = bz - az;
    return dx*dx + dy*dy + dz*dz;
}

}  // namespace

DatumGraph extractHeuristicDependencies(
    const std::vector<json>& stepParams,
    const PartsLayout&       layout,
    double                   match_tolerance_mm)
{
    DatumGraph graph;
    if (layout.parts().empty()) return graph;
    const double tol2 = match_tolerance_mm * match_tolerance_mm;

    for (size_t i = 0; i < stepParams.size(); ++i) {
        const StepPoint pt = pointFromParams(stepParams[i]);
        if (!pt.any) continue;  // step has no point-like fields

        // Find the closest part by AABB centre.
        const Part* best = nullptr;
        double      bestDist2 = std::numeric_limits<double>::infinity();
        for (const auto& part : layout.parts()) {
            const double d2 = dist2(pt.x, pt.y, pt.z,
                                    part.centerX(), part.centerY(), part.centerZ());
            if (d2 < bestDist2) {
                bestDist2 = d2;
                best      = &part;
            }
        }
        if (best && bestDist2 <= tol2) {
            graph.addDependency(static_cast<int>(i), best->id);
        }
    }
    return graph;
}

#if KOO_PARTS_HAS_REAL_PROCESS_PLAN
DatumGraph extractHeuristicDependencies(
    const process::ProcessPlan& plan,
    const PartsLayout&          layout,
    double                      match_tolerance_mm)
{
    std::vector<json> params;
    params.reserve(plan.steps().size());
    for (const auto& s : plan.steps()) params.push_back(s.params);
    return extractHeuristicDependencies(params, layout, match_tolerance_mm);
}

process::ProcessPlan reframePlanForMoves(
    const process::ProcessPlan& plan,
    const PartsLayout&          oldLayout,
    const PartsLayout&          newLayout,
    double                      match_tolerance_mm)
{
    // 1. Which step depends on which part (point-in-AABB heuristic).
    const DatumGraph graph =
        extractHeuristicDependencies(plan, oldLayout, match_tolerance_mm);

    // 2. Accumulate, per step, the total centre delta of every MOVED part it
    //    depends on (a step straddling two moved parts gets the sum).
    std::map<int, std::array<double, 3>> stepDelta;
    for (const auto& d : oldLayout.diff(newLayout)) {
        if (d.kind != "moved" || !d.details.contains("delta")) continue;
        const auto& dl = d.details["delta"];
        if (!dl.is_array() || dl.size() < 3) continue;
        const double dx = dl[0].get<double>();
        const double dy = dl[1].get<double>();
        const double dz = dl[2].get<double>();
        for (int si : graph.stepsDependentOn(d.part_id)) {
            auto& acc = stepDelta[si];
            acc[0] += dx; acc[1] += dy; acc[2] += dz;
        }
    }

    // 3. Copy the plan, shifting position fields of dependent steps.
    process::ProcessPlan out;
    const auto& steps = plan.steps();
    for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
        process::StepInvocation s = steps[i];
        auto it = stepDelta.find(i);
        if (it != stepDelta.end()) {
            shiftAxisCoord(s.params, 'x', it->second[0]);
            shiftAxisCoord(s.params, 'y', it->second[1]);
            shiftAxisCoord(s.params, 'z', it->second[2]);
        }
        out.append(s);
    }
    return out;
}
#endif

}  // namespace koocadcam::parts
