// @lat: [[engine/skills#Layer 5 LLM adapter]]

#include "FeatureTransfer.hpp"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace koocadcam::adapt {

using nlohmann::json;

// ── Internal helpers ─────────────────────────────────────────────────────

namespace {

// The positional key-prefix family — MUST mirror the set scanned by
// parts/DatumGraph.cpp's extractAxisCoord/setAxisCoord so both layers agree
// on what "a position" is.
constexpr const char* kPositionalPrefixes[] = { "position_", "center_", "offset_" };

bool endsWith(const std::string& s, const std::string& suf)
{
    return s.size() >= suf.size() &&
           s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

}  // namespace

// ── AnchorFrame ──────────────────────────────────────────────────────────

AnchorFrame AnchorFrame::fromShape(const TopoDS_Shape& shape)
{
    AnchorFrame f;
    if (shape.IsNull()) return f;

    // AddOptimal walks the actual surface (vs the NURBS control polygon
    // BRepBndLib::Add returns) — the repo-wide convention; see
    // engine/primitives/Bbox.hpp and parts/PartsLayout.cpp.
    Bnd_Box box;
    BRepBndLib::AddOptimal(shape, box);
    if (box.IsVoid()) return f;

    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    f.cx = 0.5 * (xmin + xmax);
    f.cy = 0.5 * (ymin + ymax);
    f.cz = 0.5 * (zmin + zmax);
    f.hx = 0.5 * (xmax - xmin);
    f.hy = 0.5 * (ymax - ymin);
    f.hz = 0.5 * (zmax - zmin);
    return f;
}

// ── classifyParam ────────────────────────────────────────────────────────

ParamRole classifyParam(const std::string& key)
{
    // Positional — the (position_|center_|offset_)(x|y|z)_mm family that
    // parts/DatumGraph.cpp's extractAxisCoord scans, plus the other keys
    // that encode a location in the product's frame.
    static constexpr char kAxes[] = { 'x', 'y', 'z' };
    for (const char* pre : kPositionalPrefixes) {
        for (const char axis : kAxes) {
            if (key == std::string(pre) + axis + "_mm") return ParamRole::Positional;
        }
    }
    if (key == "start_x_mm"  || key == "start_y_mm"  ||
        key == "world_ox_mm" || key == "world_oy_mm" || key == "world_oz_mm" ||
        key == "edges_at_z_mm")
        return ParamRole::Positional;

    // Datum — face/axis references selecting WHERE on the body to act.
    if (key == "entry_face" || key == "entry_face_id" ||
        key == "face_normal" || key == "axis_dir")
        return ParamRole::Datum;

    // Diagnostic — recognizer breadcrumbs, meaningless off the source part.
    if (key == "world_center" || key == "hole_centers" || key == "cyl_face_ids")
        return ParamRole::Diagnostic;

    // Intrinsic — the feature's own dimensions/counts.  Listed explicitly
    // for documentation; unrecognised keys ALSO default here (an unknown key
    // is safer preserved untouched than guessed at).
    if (endsWith(key, "_dia_mm") || endsWith(key, "_depth_mm") ||
        key == "depth_mm" || key == "taper_deg" || endsWith(key, "_angle_deg") ||
        key == "length_mm" || key == "width_mm" || key == "height_mm" ||
        key == "count" || key == "hole_count" || key == "rows" ||
        key == "cols" || key == "sections" ||
        key.find("pitch") != std::string::npos ||
        key.find("spacing") != std::string::npos ||
        key == "through_hole")
        return ParamRole::Intrinsic;

    return ParamRole::Intrinsic;
}

// ── transferFeature ──────────────────────────────────────────────────────
//
// Slice-1 VALIDATED ENVELOPE (everything outside it REFUSES loudly —
// transferred == false — instead of emitting a silently-wrong step):
//   - skill: annular_groove only;
//   - entry: the FRONT/BACK (±Z) face role; a surviving non-±Z face_normal
//     refuses (the fit clamp's axis mapping is only sound for a ±Z entry);
//   - placement: near-CONCENTRIC with the destination frame — the executed
//     position is FACE-LOCAL (annular_groove::apply offsets from the resolved
//     face's centre), which coincides with this frame math only while the
//     entry face is centred on the product frame (true of both shipped
//     products) and the offset is small; beyond half-way to the boundary the
//     approximation refuses.  Full face-anchored placement is follow-up.
//   - destination: a rectangular-footprint slab thick enough to hold the
//     groove plus 1 mm of floor.  A round-footprint destination (phone→watch)
//     is not modelled by the AABB clamp and stays out of scope.

namespace {

// A refusal must not hand back an executable copy of the source step: replace
// the skill id so any caller that ignores `transferred` fails LOUDLY at the
// Executor (no dispatch entry) instead of cutting source-frame geometry.
void neuter(TransferResult& r, const std::string& why)
{
    r.step.skill_id = "refused_transfer";
    r.notes.push_back(why);
}

}  // namespace

TransferResult transferFeature(const process::StepInvocation& src,
                               const std::string&             srcProduct,
                               const std::string&             dstProduct,
                               const AnchorFrame&             srcFrame,
                               const AnchorFrame&             dstFrame,
                               const TransferOptions&         opts)
{
    TransferResult result;
    result.step = src;

    // 1. Cross-product: indices into the SOURCE plan are meaningless on the
    //    destination — a transferred step arrives dependency-free.
    result.step.depends_on.clear();
    const std::string tag = "adapt::transferFeature " + srcProduct + "->" + dstProduct;
    result.step.note = result.step.note.empty() ? tag : result.step.note + "; " + tag;
    result.notes.push_back(tag);

    // 2. Slice-1 whitelist — explicit refusal beats silent wrongness.
    if (result.step.skill_id != "annular_groove") {
        neuter(result, "unsupported skill for cross-product transfer: '" +
                       src.skill_id + "'");
        spdlog::debug("adapt::transferFeature {}->{}: refused (skill '{}' not whitelisted)",
                      srcProduct, dstProduct, src.skill_id);
        return result;   // transferred stays false
    }

    // Degenerate frames make every downstream ratio meaningless — refuse.
    if (srcFrame.hx <= 1e-9 || srcFrame.hy <= 1e-9 ||
        dstFrame.hx <= 1e-9 || dstFrame.hy <= 1e-9 || dstFrame.hz <= 1e-9) {
        neuter(result, "degenerate source/destination anchor frame");
        return result;
    }

    if (!result.step.params.is_object()) result.step.params = json::object();
    json& p = result.step.params;

    // 3. Strip Datum + Diagnostic keys that are bound to the SOURCE product:
    //    a face id indexes the source face table, entry_face names a source
    //    datum, and the recognizer breadcrumbs are source-world coordinates.
    //    The taxonomy is classifyParam's — the same one the tests pin.
    {
        std::vector<std::string> doomed;
        for (auto it = p.begin(); it != p.end(); ++it) {
            const ParamRole role = classifyParam(it.key());
            if (role == ParamRole::Diagnostic) doomed.push_back(it.key());
            else if (role == ParamRole::Datum &&
                     (it.key() == "entry_face" || it.key() == "entry_face_id"))
                doomed.push_back(it.key());
        }
        for (const auto& k : doomed) {
            p.erase(k);
            result.notes.push_back("stripped product-bound key '" + k + "'");
        }
    }

    // 4. Datum: slice-1 transfers a FRONT/BACK-face feature.  Inject the front
    //    (+Z) normal when absent; REFUSE a surviving non-±Z normal — the fit
    //    clamp below maps depth to Z and the footprint to X/Y, which is only
    //    sound for a ±Z entry plane.
    if (!p.contains("face_normal")) {
        p["face_normal"] = json::array({ 0.0, 0.0, 1.0 });
        result.notes.push_back("injected face_normal [0,0,1] (front-face role)");
    } else if (p["face_normal"].is_array() && p["face_normal"].size() == 3 &&
               p["face_normal"][0].is_number() && p["face_normal"][1].is_number() &&
               p["face_normal"][2].is_number()) {
        const double nx = p["face_normal"][0].get<double>();
        const double ny = p["face_normal"][1].get<double>();
        const double nz = p["face_normal"][2].get<double>();
        const double n  = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (n < 1e-9 || std::abs(nz / n) < 0.999) {
            neuter(result, "non-front (non-±Z) entry face_normal — outside the "
                           "slice-1 envelope");
            return result;
        }
    } else {
        neuter(result, "malformed face_normal in source step");
        return result;
    }

    // 5. Positional scaling: every present x/y key classified Positional
    //    re-expresses in the destination frame, v' = v * (dstHalf/srcHalf).
    struct AxisMap { char axis; double srcHalf; double dstHalf; };
    const AxisMap axes[] = { { 'x', srcFrame.hx, dstFrame.hx },
                             { 'y', srcFrame.hy, dstFrame.hy } };
    for (const auto& a : axes) {
        for (const char* pre : kPositionalPrefixes) {
            const std::string key = std::string(pre) + a.axis + "_mm";
            if (!p.contains(key) || !p[key].is_number()) continue;
            if (classifyParam(key) != ParamRole::Positional) continue;
            p[key] = p[key].get<double>() * (a.dstHalf / a.srcHalf);
        }
    }
    // z positions are a slice-1 punt: entry-Z is re-established from the
    // DESTINATION face at execute time (the entryPointOnFacePlane
    // convention), not scaled from a source-product z.
    for (const char* pre : kPositionalPrefixes) {
        const std::string key = std::string(pre) + "z_mm";
        if (p.contains(key))
            result.notes.push_back("z positional key '" + key +
                                   "' present; slice-1 does not rescale z");
    }

    // 6. Intrinsics (outer_dia_mm / inner_dia_mm / depth_mm / taper_deg)
    //    are deliberately untouched — see the header policy.

    // 7. Fit checks — intrinsic preservation is the policy, but physics wins.
    //    Type-guarded reads: a malformed source param refuses, never throws.
    auto num = [&](const char* key, double dflt) -> double {
        if (!p.contains(key)) return dflt;
        if (!p[key].is_number()) return std::numeric_limits<double>::quiet_NaN();
        return p[key].get<double>();
    };
    const double cx = num("center_x_mm", 0.0);
    const double cy = num("center_y_mm", 0.0);
    double od = num("outer_dia_mm", 0.0);
    double id = num("inner_dia_mm", 0.0);
    if (std::isnan(cx) || std::isnan(cy) || std::isnan(od) || std::isnan(id)) {
        neuter(result, "malformed numeric param in source step");
        return result;
    }

    // CONCENTRICITY envelope: the executed position is FACE-LOCAL (offset from
    // the resolved face's centre), so the frame-ratio scaling above and the
    // AABB clamp below are only faithful while the ring stays near the frame
    // centre.  Past half-way to the boundary the approximation is unsafe.
    if (std::abs(cx) > 0.5 * dstFrame.hx || std::abs(cy) > 0.5 * dstFrame.hy) {
        neuter(result, "scaled ring centre beyond half the destination half-extent"
                       " — outside the slice-1 concentric envelope");
        return result;
    }

    // A ring whose groove wall is below the 0.5 mm machinable floor is refused
    // whether or not it fits — transferring an unmachinable feature is a lie.
    if (od > 1e-9 && 0.5 * (od - id) < 0.5) {
        neuter(result, "source ring groove wall " + std::to_string(0.5 * (od - id)) +
                       " mm < 0.5 mm — not machinable");
        return result;
    }

    // Depth: a groove deeper than (almost) the destination thickness would
    // sever the part; clamp to thickness minus 1 mm of floor stock.  A
    // destination too thin to hold ANY groove refuses instead of stamping a
    // nonsense depth (the transferred=true contract means "safe to execute").
    const double maxDepth = 2.0 * dstFrame.hz - 1.0;
    const double depth    = num("depth_mm", 0.0);
    if (std::isnan(depth)) {
        neuter(result, "malformed depth_mm in source step");
        return result;
    }
    if (maxDepth < 0.1) {
        neuter(result, "destination too thin for any groove (thickness "
                       + std::to_string(2.0 * dstFrame.hz) + " mm)");
        return result;
    }
    if (depth > maxDepth) {
        result.notes.push_back("depth clamped " + std::to_string(depth) + " -> " +
                               std::to_string(maxDepth) +
                               " mm (destination thickness minus 1 mm floor)");
        p["depth_mm"] = maxDepth;
        result.fit_clamped = true;
    }

    // 8. Radial: |c| + OD/2 must clear each half-extent minus the margin.
    //    When violated, shrink OD and ID by ONE factor (ratio-preserving — the
    //    groove keeps its proportions) to the largest OD that fits both axes.
    const double margin = opts.fit_margin_mm;
    const bool fitsX = std::abs(cx) + 0.5 * od <= dstFrame.hx - margin;
    const bool fitsY = std::abs(cy) + 0.5 * od <= dstFrame.hy - margin;
    if (od > 1e-9 && (!fitsX || !fitsY)) {
        const double maxOdX = (dstFrame.hx - margin - std::abs(cx)) * 2.0;
        const double maxOdY = (dstFrame.hy - margin - std::abs(cy)) * 2.0;
        const double feasibleOd = std::max(0.0, std::min(maxOdX, maxOdY));
        const double factor = feasibleOd / od;
        od *= factor;
        id *= factor;
        p["outer_dia_mm"] = od;
        p["inner_dia_mm"] = id;
        result.fit_clamped = true;
        result.notes.push_back("fit clamp: OD/ID scaled by " + std::to_string(factor) +
                               " to fit the destination footprint");
        // A clamp that leaves no machinable groove wall is a refusal, not a
        // transfer — a sub-0.5 mm ring wall cannot be cut.
        if (0.5 * (od - id) < 0.5) {
            neuter(result, "post-clamp groove wall " + std::to_string(0.5 * (od - id)) +
                           " mm < 0.5 mm — refusing transfer");
            spdlog::debug("adapt::transferFeature {}->{}: refused (post-clamp wall too thin)",
                          srcProduct, dstProduct);
            return result;   // transferred stays false
        }
    }

    // 9. Success.
    result.transferred = true;
    spdlog::debug("adapt::transferFeature {}->{}: '{}' transferred (fit_clamped={})",
                  srcProduct, dstProduct, result.step.skill_id, result.fit_clamped);
    return result;
}

}  // namespace koocadcam::adapt
