// @lat: [[engine/skills#Layer 6 Datum-part dependency graph]]

#include "DatumGraph.hpp"

#include <spdlog/spdlog.h>

#include <gp_Dir.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>

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

// Set whichever coordinate field exists for `axis` to `value` (the same prefix
// set extractAxisCoord scans).  Writes only the first matching field per axis.
void setAxisCoord(json& params, char axis, double value)
{
    if (!params.is_object()) return;
    static const std::vector<std::string> kPrefixes = {
        "position_", "center_", "offset_"
    };
    for (const auto& pre : kPrefixes) {
        const std::string key = pre + axis + "_mm";
        if (params.contains(key) && params[key].is_number()) {
            params[key] = value;
            return;
        }
    }
}

// Read the step's (x, y, z).  Returns nullopt if NO axis is found at all
// (so the step is "position-less" — not amenable to point matching).  If
// some axes are missing but others are present, the missing ones default
// to 0.0 (e.g. a 2D pocket on the top face encodes only x + y).
// A step's feature point, with per-axis presence.  A top-face drill carries
// only x,y — its z is ABSENT (hasZ == false), and must not gate matching.
struct StepPoint {
    double x = 0.0, y = 0.0, z = 0.0;
    bool   hasX = false, hasY = false, hasZ = false;
    bool   any() const { return hasX || hasY || hasZ; }
};
StepPoint pointFromParams(const json& params)
{
    StepPoint pt;
    if (auto vx = extractAxisCoord(params, 'x')) { pt.x = *vx; pt.hasX = true; }
    if (auto vy = extractAxisCoord(params, 'y')) { pt.y = *vy; pt.hasY = true; }
    if (auto vz = extractAxisCoord(params, 'z')) { pt.z = *vz; pt.hasZ = true; }
    return pt;
}

// The part that OWNS a feature point: the smallest-footprint part whose AABB,
// expanded by `margin`, contains the point on every axis the step actually
// specifies.  Containment (not nearest-centre) is the general rule — one PCB
// part can own all of its mounting holes without its centre sitting on any of
// them, and a feature's omitted axis (e.g. a top-face drill's z) is ignored so
// the part may sit anywhere along that axis.  A zero-size (point) part reduces
// to a per-axis |Δ| ≤ margin test, preserving the old point-match behaviour.
// Smallest AABB volume wins ties → the most specific (innermost) part owns it.
const Part* anchorPart(const PartsLayout& layout, const StepPoint& pt, double margin)
{
    const Part* best    = nullptr;
    double      bestVol = std::numeric_limits<double>::infinity();
    for (const auto& part : layout.parts()) {
        if (pt.hasX && (pt.x < part.xMin - margin || pt.x > part.xMax + margin)) continue;
        if (pt.hasY && (pt.y < part.yMin - margin || pt.y > part.yMax + margin)) continue;
        if (pt.hasZ && (pt.z < part.zMin - margin || pt.z > part.zMax + margin)) continue;
        const double vol = (part.sizeX() + 1e-6) *
                           (part.sizeY() + 1e-6) *
                           (part.sizeZ() + 1e-6);
        if (vol < bestVol) { bestVol = vol; best = &part; }
    }
    return best;
}

// Move a step's feature point through a part's full AFFINE pose change.  The
// part's pose is modelled as { position = AABB centre, orientation = placement
// rotation, size = AABB extents }.  The feature point f is expressed in the
// part's OLD local frame, scaled by the per-axis size ratio, then placed in the
// NEW frame:
//
//     f_local = R_old⁻¹ · (f − oldCentre)
//     f'      = newCentre + R_new · (S ⊙ f_local),   S = newSize / oldSize
//
// Special cases fall out cleanly:
//   - S = 1, R_old = R_new           → pure translation  (f' = f + centreΔ)
//   - S = 1                          → rigid rotation     (f' = newC + ΔR·(f−oldC))
//   - R_old = R_new = I              → translate + scale  (offsets grow with the part)
// Axes the step omits (e.g. a top-face drill's z) are seeded from the centre so
// a 2-D feature transforms within its plane and the absent axis is never
// written back.
// ── Dimension scaling (B9.1) ───────────────────────────────────────────────
//
// Positions transform affinely (above); a feature's DIMENSIONS must scale with
// the part too, or a grown part keeps Ø3 holes that no longer match the larger
// bolt circle / clearance.  The part's resize is anisotropic — (sx, sy, sz) in
// the part's LOCAL frame — so a dimension scales by the part stretch ALONG its
// own direction:
//
//   • length_mm / width_mm / height_mm — extents along local X / Y / Z → sx/sy/sz.
//   • depth_mm and *_depth_mm           — measured along the feature AXIS → the
//     stretch of a unit vector along that axis,  |S ⊙ â|.
//   • diameter / radius / *_dia_mm / *_r_mm / corner_r_mm — RADIAL, in the plane
//     ⊥ the axis → the mean in-plane stretch (a cylinder must stay a cylinder,
//     so we use one isotropic factor rather than turning the circle into an
//     ellipse).
//
// â is the feature's axis_dir (default local −Z, the drilling convention),
// expressed in the part's local frame so the diagonal S applies directly.  All
// of this reduces to the obvious answers for the common Z-axis feature
// (axial = sz, radial = (sx+sy)/2) and is exact for any axis + anisotropic S.
namespace {

double stretchAlong(const gp_Vec& unit, double sx, double sy, double sz)
{
    const gp_Vec s(unit.X() * sx, unit.Y() * sy, unit.Z() * sz);
    return s.Magnitude();   // |S ⊙ û| for a unit û
}

void scaleFeatureDimensions(json& params, const gp_Trsf& rOldInv,
                            double sx, double sy, double sz)
{
    if (!params.is_object()) return;
    if (std::abs(sx - 1.0) < 1e-9 && std::abs(sy - 1.0) < 1e-9 &&
        std::abs(sz - 1.0) < 1e-9) {
        return;   // pure move/rotate — no dimension change
    }

    // Feature axis in the part's LOCAL frame (default −Z drilling axis).
    gp_Vec axisLocal(0.0, 0.0, -1.0);
    if (params.contains("axis_dir") && params["axis_dir"].is_array() &&
        params["axis_dir"].size() == 3 && params["axis_dir"][0].is_number()) {
        const gp_Vec world(params["axis_dir"][0].get<double>(),
                           params["axis_dir"][1].get<double>(),
                           params["axis_dir"][2].get<double>());
        if (world.Magnitude() > 1e-9) axisLocal = world;
    }
    if (axisLocal.Magnitude() < 1e-9) axisLocal = gp_Vec(0.0, 0.0, -1.0);
    axisLocal.Normalize();
    axisLocal.Transform(rOldInv);                  // world axis → local frame (rotation only)
    if (axisLocal.Magnitude() < 1e-9) axisLocal = gp_Vec(0.0, 0.0, -1.0);
    axisLocal.Normalize();

    const double axialScale = stretchAlong(axisLocal, sx, sy, sz);

    // Two unit vectors spanning the plane ⊥ axis → mean in-plane stretch.
    gp_Vec ref = (std::abs(axisLocal.Z()) < 0.9) ? gp_Vec(0, 0, 1)
                                                 : gp_Vec(1, 0, 0);
    gp_Vec u = axisLocal.Crossed(ref);
    if (u.Magnitude() < 1e-9) u = gp_Vec(1, 0, 0); else u.Normalize();
    gp_Vec v = axisLocal.Crossed(u);
    if (v.Magnitude() < 1e-9) v = gp_Vec(0, 1, 0); else v.Normalize();
    const double radialScale =
        0.5 * (stretchAlong(u, sx, sy, sz) + stretchAlong(v, sx, sy, sz));

    auto endsWith = [](const std::string& s, const std::string& suf) {
        return s.size() >= suf.size() &&
               s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
    };

    for (auto it = params.begin(); it != params.end(); ++it) {
        if (!it.value().is_number()) continue;
        const std::string k = it.key();
        double factor = 0.0;
        if (k == "length_mm")       factor = sx;
        else if (k == "width_mm")   factor = sy;
        else if (k == "height_mm")  factor = sz;
        else if (k == "depth_mm" || endsWith(k, "_depth_mm")) factor = axialScale;
        else if (k == "diameter_mm" || k == "radius_mm" || k == "r_mm" ||
                 endsWith(k, "_dia_mm") || endsWith(k, "_r_mm") ||
                 endsWith(k, "_diameter_mm") || endsWith(k, "_radius_mm"))
            factor = radialScale;
        if (factor != 0.0) it.value() = it.value().get<double>() * factor;
    }
}

}  // namespace

void applyPartDeltaToParams(json& params, const Part& oldP, const Part& newP)
{
    const StepPoint pt = pointFromParams(params);

    const gp_XYZ oldC(oldP.centerX(), oldP.centerY(), oldP.centerZ());
    const gp_XYZ newC(newP.centerX(), newP.centerY(), newP.centerZ());

    gp_Trsf rOldInv; rOldInv.SetRotation(oldP.placement.GetRotation().Inverted());
    gp_Trsf rNew;    rNew.SetRotation(newP.placement.GetRotation());

    auto ratio = [](double num, double den) {
        return (std::abs(den) > 1e-9) ? (num / den) : 1.0;
    };
    const double sx = ratio(newP.sizeX(), oldP.sizeX());
    const double sy = ratio(newP.sizeY(), oldP.sizeY());
    const double sz = ratio(newP.sizeZ(), oldP.sizeZ());

    // Dimensions scale with the part regardless of whether it carries a point.
    scaleFeatureDimensions(params, rOldInv, sx, sy, sz);

    if (!pt.any()) return;   // position-less step: dims done, nothing to move

    gp_XYZ rel(pt.hasX ? pt.x : oldP.centerX(),
               pt.hasY ? pt.y : oldP.centerY(),
               pt.hasZ ? pt.z : oldP.centerZ());
    rel -= oldC;
    rOldInv.Transforms(rel);                       // into old local axes
    rel.SetCoord(rel.X() * sx, rel.Y() * sy, rel.Z() * sz);  // scale offsets
    rNew.Transforms(rel);                          // into new orientation
    const gp_XYZ res = newC + rel;

    if (pt.hasX) setAxisCoord(params, 'x', res.X());
    if (pt.hasY) setAxisCoord(params, 'y', res.Y());
    if (pt.hasZ) setAxisCoord(params, 'z', res.Z());
}

}  // namespace

DatumGraph extractHeuristicDependencies(
    const std::vector<json>& stepParams,
    const PartsLayout&       layout,
    double                   match_tolerance_mm)
{
    DatumGraph graph;
    if (layout.parts().empty()) return graph;

    for (size_t i = 0; i < stepParams.size(); ++i) {
        const StepPoint pt = pointFromParams(stepParams[i]);
        if (!pt.any()) continue;  // step has no point-like fields
        if (const Part* owner = anchorPart(layout, pt, match_tolerance_mm)) {
            graph.addDependency(static_cast<int>(i), owner->id);
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
    // 1. Which step is owned by which part (AABB-containment heuristic).
    const DatumGraph graph =
        extractHeuristicDependencies(plan, oldLayout, match_tolerance_mm);

    // 2. Copy the plan; for each owned step, move its feature point through the
    //    owner part's rigid-body pose change (translation + rotation).  A part
    //    whose pose is unchanged yields an identity delta, so its features stay
    //    put — only the parts that actually moved/rotated drag their features.
    process::ProcessPlan out;
    const auto& steps = plan.steps();
    for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
        process::StepInvocation s = steps[i];
        const std::vector<std::string> owners = graph.partsForStep(i);
        if (!owners.empty()) {
            const Part* oldP = oldLayout.findById(owners.front());
            const Part* newP = newLayout.findById(owners.front());
            if (oldP && newP) applyPartDeltaToParams(s.params, *oldP, *newP);
        }
        out.append(s);
    }
    return out;
}
#endif

}  // namespace koocadcam::parts
