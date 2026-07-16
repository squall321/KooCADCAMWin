// @lat: [[engine/skills#Layer 5 LLM adapter]]

#include "FeatureTransfer.hpp"

#include "skills/Datum.hpp"
#include "skills/Workpiece.hpp"
#include "skills/countersink.hpp"

#include <BRepBndLib.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <Bnd_Box.hxx>
#include <TopAbs_State.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
        key == "origin_x_mm" || key == "origin_y_mm" ||
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
// VALIDATED ENVELOPE (everything outside it REFUSES loudly —
// transferred == false — instead of emitting a silently-wrong step); the
// full family/placement/clamp policy lives in the header:
//   - skills: annular_groove (FACE-LOCAL placement) + the WORLD-XY
//     pitch-circle hole patterns (bolt_circle / counterbore_ring /
//     countersink_ring — see PatternFamily below);
//   - entry: the FRONT/BACK (±Z) face role; a non-±Z face_normal or
//     pattern axis_dir refuses (the fit clamp's axis mapping is only sound
//     for a ±Z entry);
//   - placement: EXACT — face-anchored when opts.dst_shape is supplied; the
//     face-local family without it falls back to the conservative
//     near-concentric envelope (the pattern family's world placement needs
//     no face and is exact in both modes);
//   - destination footprint: face-anchored mode MEASURES it (the outermost
//     circle is classified against the actual solid at the entry plane —
//     round watch cases, stepped decks, lug/crown protrusions and existing
//     cuts are all handled by interrogation, not modelling); frame-only mode
//     keeps the AABB approximation, so a round-footprint destination is only
//     sound face-anchored;
//   - destination thick enough to hold the cut plus 1 mm of floor
//     (through-drilled bolt circles are exempt).

namespace {

// A refusal must not hand back an executable copy of the source step: replace
// the skill id so any caller that ignores `transferred` fails LOUDLY at the
// Executor (no dispatch entry) instead of cutting source-frame geometry.
void neuter(TransferResult& r, const std::string& why)
{
    r.step.skill_id = "refused_transfer";
    r.notes.push_back(why);
}

// The WORLD-XY pattern family: hole patterns on a pitch circle whose
// center_x/y are WORLD coordinates (the member atoms pierce the entry plane
// at world (px, py) — skills/Workpiece.hpp entryPointOnFacePlane) and whose
// members apply() recomputes from centre + PCD + count + start angle.  The
// keys below are the family's per-skill vocabulary; everything else the
// transfer needs (bolt_circle_dia_mm, center_x/y_mm, start_angle_deg,
// axis_dir) is shared verbatim across the three skills.
struct PatternFamily
{
    const char* count_key;    // member count ("count" / "hole_count")
    const char* widest_key;   // the WIDEST member cut dia — gates footprint
    const char* depth_key;    // the DEEPEST cut, measured from the entry plane
};

const PatternFamily* patternFamily(const std::string& skillId)
{
    static const std::unordered_map<std::string, PatternFamily> kFamilies = {
        { "bolt_circle_pattern",      { "hole_count", "hole_dia_mm",     "depth_mm" } },
        { "counterbore_ring_pattern", { "count",      "seat_dia_mm",     "pilot_depth_mm" } },
        { "countersink_ring_pattern", { "count",      "cone_top_dia_mm", "pilot_depth_mm" } },
    };
    const auto it = kFamilies.find(skillId);
    return it == kFamilies.end() ? nullptr : &it->second;
}

// MEASURED radial feasibility (face-anchored mode): every sample on the
// circle of radius r about (cx, cy) at height z must classify IN (or ON)
// the destination solid.  The sample count scales with the circle so
// adjacent probes stay <= ~0.4 mm apart (floor 48): the finest shipped
// face-open voids — the phone's 0.5 mm-wide camera deco-groove band, the
// watch's Ø1.5 rear sensor recess — cannot slip between probes.  (A fixed
// 48 left ~2.6 mm gaps on a Ø40 circle, wider than both.)
bool circleInMaterial(BRepClass3d_SolidClassifier& cls,
                      double cx, double cy, double z, double r)
{
    const int n = std::max(48, static_cast<int>(std::ceil(2.0 * M_PI * r / 0.4)));
    for (int i = 0; i < n; ++i) {
        const double a = 2.0 * M_PI * i / n;
        cls.Perform(gp_Pnt(cx + r * std::cos(a), cy + r * std::sin(a), z), 1e-6);
        const TopAbs_State st = cls.State();
        if (st != TopAbs_IN && st != TopAbs_ON) return false;
    }
    return true;
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

    // 2. Whitelist — explicit refusal beats silent wrongness.  Three
    //    placement families (see the header): FACE-LOCAL annular_groove, the
    //    WORLD-XY pitch-circle hole patterns, and the WORLD-XY linear hole
    //    array (start/direction/pitch instead of a pitch circle).
    const PatternFamily* fam = patternFamily(result.step.skill_id);
    const bool linear = result.step.skill_id == "linear_hole_array";
    const bool grid   = result.step.skill_id == "rectangular_hole_grid";
    if (result.step.skill_id != "annular_groove" && fam == nullptr &&
        !linear && !grid) {
        neuter(result, "unsupported skill for cross-product transfer: '" +
                       src.skill_id + "'");
        spdlog::debug("adapt::transferFeature {}->{}: refused (skill '{}' not whitelisted)",
                      srcProduct, dstProduct, src.skill_id);
        return result;   // transferred stays false
    }
    // WORLD-XY placement family (frame-relative re-expression + world ecc).
    const bool worldXY = fam != nullptr || linear || grid;

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

    // 4. Datum: a FRONT/BACK-face feature only — the fit clamp below maps
    //    depth to Z and the footprint to X/Y, which is only sound for a ±Z
    //    entry plane.  Each family carries its portable datum differently:
    //      face-local — face_normal (the entry face's normal, injected +Z
    //                    when absent);
    //      pattern    — axis_dir (the DRILLING direction; the entry face is
    //                    the one facing OPPOSITE it — exactly the Executor's
    //                    by-normal fallback once entry_face_id is stripped).
    //    nzEntry is the ENTRY-FACE normal sign either way; the face-anchored
    //    resolution and the depth limit below key off it.
    double nzEntry  = 1.0;
    gp_Dir entryDir(0.0, 0.0, 1.0);   // the exact dir the Executor will resolve
    if (!worldXY) {
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
                               "validated envelope");
                return result;
            }
        } else {
            neuter(result, "malformed face_normal in source step");
            return result;
        }
        nzEntry  = p["face_normal"][2].get<double>() >= 0.0 ? 1.0 : -1.0;
        entryDir = gp_Dir(0.0, 0.0, nzEntry);
    } else {
        if (!p.contains("axis_dir") || !p["axis_dir"].is_array() ||
            p["axis_dir"].size() != 3 ||
            !p["axis_dir"][0].is_number() || !p["axis_dir"][1].is_number() ||
            !p["axis_dir"][2].is_number()) {
            neuter(result, "pattern step without a well-formed axis_dir");
            return result;
        }
        const double ax = p["axis_dir"][0].get<double>();
        const double ay = p["axis_dir"][1].get<double>();
        const double az = p["axis_dir"][2].get<double>();
        const double a  = std::sqrt(ax * ax + ay * ay + az * az);
        // Mirror the pattern skills' own validate error (|axis Z| >= 0.99):
        // transferred=true must mean apply() will not throw on the axis.
        if (a < 1e-9 || std::abs(az / a) < 0.99) {
            neuter(result, "non-vertical pattern axis_dir — the pattern skills "
                           "support a ±Z axis only");
            return result;
        }
        nzEntry  = az > 0.0 ? -1.0 : 1.0;      // entry face opposes the drill
        entryDir = gp_Dir(-ax, -ay, -az);      // parseXxxRingPattern parity
    }

    // 5. Positional scaling: every present x/y key classified Positional
    //    re-expresses in the destination frame.  The two families store
    //    positions differently:
    //      face-local — offsets from the (resolved) entry-face centre; a bare
    //                   ratio re-expresses them: v' = v * (dstHalf/srcHalf);
    //      pattern    — WORLD coordinates; they re-express RELATIVE to the
    //                   frame centres: v' = dstC + (v − srcC) * (dstHalf/srcHalf)
    //                   (a bare ratio would be wrong the moment a product is
    //                   not modelled about the world origin).
    struct AxisMap { char axis; double srcC, dstC, srcHalf, dstHalf; };
    const AxisMap axes[] = {
        { 'x', srcFrame.cx, dstFrame.cx, srcFrame.hx, dstFrame.hx },
        { 'y', srcFrame.cy, dstFrame.cy, srcFrame.hy, dstFrame.hy } };
    for (const auto& a : axes) {
        // kPositionalPrefixes plus the linear array's start_ / the grid's
        // origin_ families (Positional keys by classifyParam, but not
        // shared prefixes).
        static const char* kScaledPrefixes[] =
            { "position_", "center_", "offset_", "start_", "origin_" };
        for (const char* pre : kScaledPrefixes) {
            const std::string key = std::string(pre) + a.axis + "_mm";
            if (!p.contains(key) || !p[key].is_number()) continue;
            if (classifyParam(key) != ParamRole::Positional) continue;
            const double v = p[key].get<double>();
            p[key] = worldXY
                         ? a.dstC + (v - a.srcC) * (a.dstHalf / a.srcHalf)
                         : v * (a.dstHalf / a.srcHalf);
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

    // FACE-ANCHORED placement (optional): resolve the entry face NOW with the
    // exact rule the Executor will use (parseAnnularGroove's FaceByNormal from
    // face_normal / the ring parsers' by-normal fallback from −axis_dir; both
    // 5°, "largest"), so the depth limit below measures the REAL entry plane.
    // The executed XY differs per family:
    //   face-local — face centre + (cx, ±cy): apply()'s in-plane basis for a
    //                ±Z normal is vx = +X, vy = n × vx (world Y flips for a
    //                −Z entry), so anchoring needs the resolved face;
    //   pattern    — (cx, cy) IS the executed world centre (the atoms pierce
    //                the entry plane at world XY), exact with or without the
    //                shape — anchoring adds the real entry-plane Z and the
    //                face-existence refusal.
    bool   faceAnchored = false;
    double eccX = fam != nullptr ? cx - dstFrame.cx : 0.0;   // executed centre,
    double eccY = fam != nullptr ? cy - dstFrame.cy : 0.0;   // frame-relative
    double faceZ = 0.0;
    if (opts.dst_shape != nullptr) {
        try {
            skill::Workpiece dstWp(*opts.dst_shape);
            const auto fid =
                dstWp.resolve(skill::FaceByNormal{ entryDir, 5.0, "largest" });
            if (!fid) {
                neuter(result, "destination entry face (±Z, largest) unresolvable");
                return result;
            }
            const gp_Pnt C = dstWp.faceCenter(*fid);
            if (!worldXY) {
                const double ySign = nzEntry;   // vy = n × vx flips for −Z
                eccX = (C.X() + cx) - dstFrame.cx;
                eccY = (C.Y() + ySign * cy) - dstFrame.cy;
            }
            faceZ = C.Z();
            faceAnchored = true;
            result.notes.push_back(
                "face-anchored: entry face centre (" + std::to_string(C.X()) +
                ", " + std::to_string(C.Y()) + ", " + std::to_string(C.Z()) + ")");
        } catch (...) {
            neuter(result, "destination face resolution threw — refusing");
            return result;
        }
    }

    // CONCENTRICITY envelope (face-local family, frame-only mode): the
    // executed position is an offset from the resolved face's centre, so
    // without the destination shape the frame-ratio scaling and the AABB clamp
    // are only faithful while the ring stays near the frame centre.  Face-
    // anchored mode replaces this approximation with the exact placement test
    // below; the pattern family's world placement is exact in BOTH modes.
    if (fam == nullptr && !faceAnchored &&
        (std::abs(cx) > 0.5 * dstFrame.hx || std::abs(cy) > 0.5 * dstFrame.hy)) {
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

    // Depth: a cut deeper than (almost) the stock below the entry plane would
    // sever the part; clamp to that minus 1 mm of floor stock.  Face-anchored
    // mode measures the REAL stock under the resolved face (a recessed deck
    // has less than the bbox thickness — the review's measured case: the
    // phone's display floor sits 0.6 mm below the bbox top); the frame-only
    // mode conservatively assumes the bbox thickness.  A destination too thin
    // to hold ANY cut refuses instead of stamping a nonsense depth (the
    // transferred=true contract means "safe to execute").
    //
    // A THROUGH-drilled bolt circle / linear array / hole grid skips the
    // clamp entirely — through is through, whatever the destination thickness
    // (depth_mm is ignored by the skill; the Executor parsers default
    // through_hole to true, mirrored here).
    const bool throughPattern =
        (result.step.skill_id == "bolt_circle_pattern" || linear || grid) &&
        (p.contains("through_hole") && p["through_hole"].is_boolean()
             ? p["through_hole"].get<bool>()
             : true);
    const double maxDepth =
        (faceAnchored
             ? (nzEntry > 0.0 ? faceZ - (dstFrame.cz - dstFrame.hz)
                              : (dstFrame.cz + dstFrame.hz) - faceZ)
             : 2.0 * dstFrame.hz) - 1.0;
    const char*  depthKey = fam != nullptr ? fam->depth_key : "depth_mm";
    const double depth    = num(depthKey, 0.0);
    if (std::isnan(depth)) {
        neuter(result, std::string("malformed ") + depthKey + " in source step");
        return result;
    }
    // Source-intrinsic gates + the structural depth floor.  transferred=true
    // promises apply() will not throw on the destination, so every validate()
    // ERROR of the pattern skills that the transfer does not otherwise
    // guarantee is mirrored here — a recovered step CAN carry values the
    // skills refuse to re-synthesise (a real Ø0.5 micro-drilled ring exists
    // in metal, but counterbore_ring_pattern::apply hard-throws below 0.8).
    double      structuralFloor = 0.0;   // the depth clamp may not go below it
    const char* structuralWhat  = nullptr;
    if (result.step.skill_id == "counterbore_ring_pattern") {
        const double pilot = num("pilot_dia_mm", 0.0);
        const double seat  = num("seat_dia_mm", 0.0);
        const double seatD = num("seat_depth_mm", 0.0);
        if (std::isnan(pilot) || std::isnan(seat) || std::isnan(seatD)) {
            neuter(result, "malformed counterbore params in source step");
            return result;
        }
        if (pilot < 0.8 || seat <= pilot || seatD <= 0.0 || depth <= seatD) {
            neuter(result, "source counterbore ring violates the skill's own "
                           "validate gates (pilot >= 0.8, seat > pilot, "
                           "0 < seat depth < pilot depth) — apply() would throw");
            return result;
        }
        structuralFloor = seatD;
        structuralWhat  = "counterbore seat";
    } else if (result.step.skill_id == "countersink_ring_pattern") {
        skill::countersink::Input atom;
        atom.pilot_dia_mm    = num("pilot_dia_mm", 0.0);
        atom.pilot_depth_mm  = depth;
        atom.cone_top_dia_mm = num("cone_top_dia_mm", 0.0);
        atom.cone_angle_deg  = num("cone_angle_deg", 90.0);
        if (std::isnan(atom.pilot_dia_mm) || std::isnan(atom.cone_top_dia_mm) ||
            std::isnan(atom.cone_angle_deg)) {
            neuter(result, "malformed countersink cone params in source step");
            return result;
        }
        const double coneDep = skill::countersink::computeConeDepth(atom);
        if (atom.pilot_dia_mm < 0.8 || atom.cone_top_dia_mm <= atom.pilot_dia_mm ||
            atom.cone_angle_deg < 45.0 || atom.cone_angle_deg > 120.0 ||
            (coneDep > 0.0 && coneDep >= depth)) {
            neuter(result, "source countersink ring violates the skill's own "
                           "validate gates (pilot >= 0.8, cone top > pilot, "
                           "angle in [45, 120], cone shallower than pilot) — "
                           "apply() would throw");
            return result;
        }
        structuralFloor = coneDep;
        structuralWhat  = "countersink cone";
    } else if (result.step.skill_id == "bolt_circle_pattern") {
        // The composed drill_hole atom's own gates: bolt_circle_pattern::
        // validate does not carry them (unlike its ring siblings), but its
        // apply() composes drill_hole::apply per member, which hard-throws
        // below the Ø0.8 DFM-002 floor and on a blind depth <= 0.
        const double hole = num("hole_dia_mm", 0.0);
        if (std::isnan(hole)) {
            neuter(result, "malformed hole_dia_mm in source step");
            return result;
        }
        if (hole < 0.8 || (!throughPattern && depth <= 0.0)) {
            neuter(result, "source bolt circle violates the composed drill "
                           "atom's gates (hole dia >= 0.8, blind depth > 0) — "
                           "apply() would throw");
            return result;
        }
    } else if (linear) {
        // Same composed drill_hole atom as bolt_circle, plus the array's own
        // validate errors (count >= 3, positive dia/pitch, non-zero direction).
        const double hole  = num("hole_dia_mm", 0.0);
        const double pitch = num("pitch_mm", 0.0);
        const double cnt   = num("hole_count", 0.0);
        if (std::isnan(hole) || std::isnan(pitch) || std::isnan(cnt)) {
            neuter(result, "malformed linear array param in source step");
            return result;
        }
        double ux = 1.0, uy = 0.0;
        if (p.contains("direction") && p["direction"].is_array() &&
            p["direction"].size() >= 2 &&
            p["direction"][0].is_number() && p["direction"][1].is_number()) {
            ux = p["direction"][0].get<double>();
            uy = p["direction"][1].get<double>();
        }
        if (static_cast<int>(cnt) < 3 || hole < 0.8 || pitch <= 0.0 ||
            std::hypot(ux, uy) < 1e-9 || (!throughPattern && depth <= 0.0)) {
            neuter(result, "source linear array violates the skill's validate "
                           "gates or the composed drill atom's (count >= 3, "
                           "hole dia >= 0.8, pitch > 0, non-zero direction, "
                           "blind depth > 0) — apply() would throw");
            return result;
        }
    } else if (grid) {
        // Grid validate errors (cols, rows >= 2, cols*rows >= 6, positive
        // dia/pitches, non-zero basis) + the composed drill atom's gates.
        const double hole  = num("hole_dia_mm", 0.0);
        const double pu    = num("pitch_u_mm", 0.0);
        const double pv    = num("pitch_v_mm", 0.0);
        const double colsD = num("cols", 0.0);
        const double rowsD = num("rows", 0.0);
        if (std::isnan(hole) || std::isnan(pu) || std::isnan(pv) ||
            std::isnan(colsD) || std::isnan(rowsD)) {
            neuter(result, "malformed hole-grid param in source step");
            return result;
        }
        auto basisOk = [&](const char* key) {
            if (!p.contains(key)) return true;   // parse defaults are non-zero
            if (!p[key].is_array() || p[key].size() < 2 ||
                !p[key][0].is_number() || !p[key][1].is_number()) return true;
            return std::hypot(p[key][0].get<double>(),
                              p[key][1].get<double>()) >= 1e-9;
        };
        const int cols = static_cast<int>(colsD), rows = static_cast<int>(rowsD);
        if (cols < 2 || rows < 2 || cols * rows < 6 || hole < 0.8 ||
            pu <= 0.0 || pv <= 0.0 || !basisOk("u_dir") || !basisOk("v_dir") ||
            (!throughPattern && depth <= 0.0)) {
            neuter(result, "source hole grid violates the skill's validate "
                           "gates or the composed drill atom's (cols, rows >= "
                           "2, >= 6 holes, hole dia >= 0.8, pitches > 0, "
                           "non-zero basis, blind depth > 0) — apply() would "
                           "throw");
            return result;
        }
    }

    if (!throughPattern) {
        if (maxDepth < 0.1) {
            neuter(result, "destination too thin for any cut below the entry plane");
            return result;
        }
        if (depth > maxDepth) {
            // The clamp may not eat a FASTENER-CRITICAL intrinsic: a
            // counterbore whose pilot ends at or above its seat, or a
            // countersink whose pilot ends inside its cone, is a validate
            // ERROR (apply() throws) — and dimensionally it is no longer the
            // feature being transferred.  Refuse instead.
            if (structuralWhat != nullptr && maxDepth <= structuralFloor) {
                neuter(result, std::string("destination stock below the entry "
                               "plane cannot hold the ") + structuralWhat +
                               " — refusing instead of altering a "
                               "fastener-critical intrinsic");
                return result;
            }
            result.notes.push_back("depth clamped " + std::to_string(depth) +
                                   " -> " + std::to_string(maxDepth) +
                                   " mm (stock below the entry plane minus 1 mm floor)");
            p[depthKey] = maxDepth;
            result.fit_clamped = true;
        }
    }

    // 8. Radial: the feature's outermost circle — footprint radius rOut plus
    //    the fit margin — must lie inside the destination footprint.
    //
    //    rOut per family: OD/2 (face-local groove) or PCD/2 + widest-member-
    //    dia/2 (pattern).  The eccentricity is the executed centre relative
    //    to the frame centre (face-local frame-only mode falls back to the
    //    centred-face assumption, |offset| alone).
    //
    //    FEASIBILITY per mode:
    //      face-anchored — MEASURED: every sample of the outer circle at the
    //        entry plane (0.1 mm into material) must classify IN the actual
    //        solid.  This replaces the AABB approximation entirely: a round
    //        watch case clamps against its rim (an AABB over-allows up to
    //        √2× on diagonal offsets), lugs/crown that inflate the bbox do
    //        not inflate the footprint, and a circle over an EXISTING cut's
    //        void also fails — transferring onto occupied real estate is not
    //        a clean transfer.  The clamp bisects the largest feasible
    //        radius; that search assumes the footprint is star-shaped about
    //        the executed centre (true of every shipped product).
    //      frame-only — the AABB per-axis test (no shape to interrogate).
    //
    //    CLAMP application per family: the groove shrinks OD and ID by ONE
    //    factor (ratio-preserving — it keeps its proportions); the pattern
    //    shrinks the PITCH CIRCLE alone (newPcd = 2·feasibleR − widest) —
    //    the member diameters are fastener sizes, the mating identity being
    //    transferred, and stay preserved.  Post-clamp refusals: a groove
    //    wall < 0.5 mm; a PCD at or below the widest member (the skill's own
    //    validate error); a member-to-member chord below the widest dia
    //    (adjacent cuts would merge — no longer a pattern).
    const double margin = opts.fit_margin_mm;
    const double offX = fam != nullptr ? std::abs(eccX)
                                       : (faceAnchored ? std::abs(eccX) : std::abs(cx));
    const double offY = fam != nullptr ? std::abs(eccY)
                                       : (faceAnchored ? std::abs(eccY) : std::abs(cy));

    // Pattern intrinsics (validated before the fit so a degenerate source
    // refuses whether or not it fits).
    double pcd = 0.0, widest = 0.0;
    int    count = 0;
    if (fam != nullptr) {
        pcd    = num("bolt_circle_dia_mm", 0.0);
        widest = num(fam->widest_key, 0.0);
        const double countD = num(fam->count_key, 0.0);
        if (std::isnan(pcd) || std::isnan(widest) || std::isnan(countD)) {
            neuter(result, "malformed pattern param in source step");
            return result;
        }
        count = static_cast<int>(countD);
        if (count < 3 || pcd <= widest || widest <= 0.0) {
            // The skills' own validate errors (count >= 3, positive dias,
            // pitch circle wider than the widest member) — apply() would
            // throw, so a source violating them refuses up front.
            neuter(result, "degenerate pattern (count < 3, non-positive "
                           "diameters, or pitch circle <= widest member) — "
                           "the skill's validate rejects it");
            return result;
        }
    }

    // MEMBER-BASED fit (linear array / rectangular grid): every member's
    // circle (dia/2 + margin) is tested individually — a bounding circle
    // would be laterally over-conservative for a long row or a wide grid.
    // When violated, the PITCH(ES) shrink by ONE scale s about the pattern
    // CENTRE — member dia and count/cols/rows are preserved (the holes are
    // what they are; the pattern's spread adapts, keeping a grid's aspect)
    // and start/origin move in so the centre stays put.  Refuse when the
    // clamped minimum pitch would merge adjacent holes (pitch < dia).  The
    // bisection assumes the footprint is star-shaped about the pattern
    // centre, like the circular clamp.  (od is 0 here, so the circle-based
    // fit below self-skips.)
    if (linear || grid) {
        const double dia = num("hole_dia_mm", 0.0);      // gated >= 0.8 above

        // A guarded 2-vector read with parse-default parity.
        auto vec2 = [&](const char* key, double defX, double defY,
                        double& outX, double& outY) {
            outX = defX; outY = defY;
            if (p.contains(key) && p[key].is_array() && p[key].size() >= 2 &&
                p[key][0].is_number() && p[key][1].is_number()) {
                outX = p[key][0].get<double>();
                outY = p[key][1].get<double>();
            }
        };

        // Member offsets about the pattern centre (at s = 1), the centre
        // itself, and the anchor key the clamp rewrites.
        std::vector<std::pair<double, double>> offs;
        double ccx = 0.0, ccy = 0.0;
        double minPitch = 0.0;                           // merge gate basis
        if (linear) {
            const double pitch = num("pitch_mm", 0.0);   // gated > 0 above
            const int    nArr  = static_cast<int>(num("hole_count", 0.0));
            const double sx    = num("start_x_mm", 0.0);
            const double sy    = num("start_y_mm", 0.0);
            if (std::isnan(sx) || std::isnan(sy)) {
                neuter(result, "malformed linear array start in source step");
                return result;
            }
            double ux, uy;
            vec2("direction", 1.0, 0.0, ux, uy);
            const double ul = std::hypot(ux, uy);        // gated non-zero above
            ux /= ul; uy /= ul;
            const double halfSpan = 0.5 * (nArr - 1) * pitch;
            ccx = sx + ux * halfSpan;
            ccy = sy + uy * halfSpan;
            offs.reserve(static_cast<std::size_t>(nArr));
            for (int i = 0; i < nArr; ++i) {
                const double t = (i - 0.5 * (nArr - 1)) * pitch;
                offs.emplace_back(ux * t, uy * t);
            }
            minPitch = pitch;
        } else {
            const double pu   = num("pitch_u_mm", 0.0);  // gated > 0 above
            const double pv   = num("pitch_v_mm", 0.0);
            const int    cols = static_cast<int>(num("cols", 0.0));
            const int    rows = static_cast<int>(num("rows", 0.0));
            const double ox   = num("origin_x_mm", 0.0);
            const double oy   = num("origin_y_mm", 0.0);
            if (std::isnan(ox) || std::isnan(oy)) {
                neuter(result, "malformed hole-grid origin in source step");
                return result;
            }
            double ux, uy, vx, vy;
            vec2("u_dir", 1.0, 0.0, ux, uy);
            vec2("v_dir", 0.0, 1.0, vx, vy);
            const double ul = std::hypot(ux, uy), vl = std::hypot(vx, vy);
            ux /= ul; uy /= ul; vx /= vl; vy /= vl;      // gated non-zero above
            const double hu = 0.5 * (cols - 1) * pu, hv = 0.5 * (rows - 1) * pv;
            ccx = ox + ux * hu + vx * hv;
            ccy = oy + uy * hu + vy * hv;
            offs.reserve(static_cast<std::size_t>(cols * rows));
            for (int i = 0; i < cols; ++i) {
                const double tu = (i - 0.5 * (cols - 1)) * pu;
                for (int j = 0; j < rows; ++j) {
                    const double tv = (j - 0.5 * (rows - 1)) * pv;
                    offs.emplace_back(ux * tu + vx * tv, uy * tu + vy * tv);
                }
            }
            // The minimum ADJACENT member spacing over the whole lattice: the
            // grammar's fitGridXY only requires a NON-COLLINEAR basis, so a
            // legitimately skewed grid's closest neighbours can sit on the
            // short cross-diagonal, well under min(pu, pv) — a merge gate on
            // the pitches alone would pass a clamp that physically fuses them.
            minPitch = std::min(pu, pv);
            minPitch = std::min(minPitch,
                                std::hypot(pu * ux - pv * vx, pu * uy - pv * vy));
            minPitch = std::min(minPitch,
                                std::hypot(pu * ux + pv * vx, pu * uy + pv * vy));
        }

        const double rMember = 0.5 * dia + margin;
        std::unique_ptr<BRepClass3d_SolidClassifier> clsPtr;
        if (faceAnchored)
            clsPtr = std::make_unique<BRepClass3d_SolidClassifier>(*opts.dst_shape);
        const double probeZ = faceAnchored ? faceZ - nzEntry * 0.1 : 0.0;

        auto membersFit = [&](double s) {
            for (const auto& o : offs) {
                const double mx = ccx + s * o.first, my = ccy + s * o.second;
                if (clsPtr != nullptr) {
                    if (!circleInMaterial(*clsPtr, mx, my, probeZ, rMember))
                        return false;
                } else {
                    if (std::abs(mx - dstFrame.cx) + rMember > dstFrame.hx ||
                        std::abs(my - dstFrame.cy) + rMember > dstFrame.hy)
                        return false;
                }
            }
            return true;
        };

        if (!membersFit(1.0)) {
            double s = 0.0;
            if (membersFit(0.0)) {
                double lo = 0.0, hi = 1.0;
                for (int i = 0; i < 24; ++i) {
                    const double mid = 0.5 * (lo + hi);
                    if (membersFit(mid)) lo = mid; else hi = mid;
                }
                s = lo;
            }
            if (minPitch * s < dia) {
                neuter(result, "post-clamp pitch " + std::to_string(minPitch * s) +
                               " mm < hole dia " + std::to_string(dia) +
                               " mm — adjacent holes would merge");
                spdlog::debug("adapt::transferFeature {}->{}: refused (pattern merges)",
                              srcProduct, dstProduct);
                return result;
            }
            if (minPitch * s - dia < 1.5)
                result.notes.push_back("post-clamp hole gap " +
                                       std::to_string(minPitch * s - dia) +
                                       " mm is inside the DFM-003 density "
                                       "warning band");
            if (linear) {
                const double pitch = num("pitch_mm", 0.0);
                const int    nArr  = static_cast<int>(num("hole_count", 0.0));
                p["pitch_mm"]   = pitch * s;
                p["start_x_mm"] = ccx + s * offs.front().first;
                p["start_y_mm"] = ccy + s * offs.front().second;
                if (p.contains("span_mm"))
                    p["span_mm"] = (nArr - 1) * pitch * s;   // keep it true
            } else {
                p["pitch_u_mm"]  = num("pitch_u_mm", 0.0) * s;
                p["pitch_v_mm"]  = num("pitch_v_mm", 0.0) * s;
                p["origin_x_mm"] = ccx + s * offs.front().first;
                p["origin_y_mm"] = ccy + s * offs.front().second;
            }
            result.fit_clamped = true;
            result.notes.push_back("fit clamp: pitch scaled by " +
                                   std::to_string(s) +
                                   " about the pattern centre (hole dia and "
                                   "count preserved)");
        }
    }

    const double rOut = fam != nullptr ? 0.5 * (pcd + widest) : 0.5 * od;
    bool   violated  = false;
    double feasibleR = rOut;
    if (rOut > 1e-9) {
        if (faceAnchored && fam != nullptr) {
            // PER-MEMBER (like linear/grid): the ring's members land at
            // start_angle + i·360/count on the pitch circle, so test the
            // material exactly there — a member over a side cutout is
            // caught, and the empty arc between members no longer
            // over-constrains.  The frame-only path below keeps the
            // rotation-invariant outer-circle bound instead: an UNMEASURED
            // clamp must not swing with recognition noise in the phase.
            BRepClass3d_SolidClassifier cls(*opts.dst_shape);
            const double probeZ = faceZ - nzEntry * 0.1;
            const double execX  = dstFrame.cx + eccX;
            const double execY  = dstFrame.cy + eccY;
            double startDeg = num("start_angle_deg", 0.0);
            if (std::isnan(startDeg)) startDeg = 0.0;   // parse default parity
            const double rm = 0.5 * widest + margin;
            auto ringFits = [&](double rr) {
                for (int i = 0; i < count; ++i) {
                    const double a =
                        (startDeg + i * 360.0 / count) * M_PI / 180.0;
                    if (!circleInMaterial(cls, execX + rr * std::cos(a),
                                          execY + rr * std::sin(a), probeZ, rm))
                        return false;
                }
                return true;
            };
            if (!ringFits(0.5 * pcd)) {
                violated = true;
                double rFeas = 0.0;
                if (ringFits(0.0)) {
                    double lo = 0.0, hi = 0.5 * pcd;
                    for (int i = 0; i < 24; ++i) {
                        const double mid = 0.5 * (lo + hi);
                        if (ringFits(mid)) lo = mid; else hi = mid;
                    }
                    rFeas = lo;
                }
                // The clamp below derives newPcd = 2*feasibleR − widest;
                // express the measured pitch-circle radius in those terms.
                feasibleR = rFeas + 0.5 * widest;
                result.notes.push_back(
                    "measured footprint: largest feasible pitch radius " +
                    std::to_string(rFeas) + " mm at the entry plane");
            }
        } else if (faceAnchored) {
            BRepClass3d_SolidClassifier cls(*opts.dst_shape);
            const double probeZ = faceZ - nzEntry * 0.1;
            const double execX  = dstFrame.cx + eccX;
            const double execY  = dstFrame.cy + eccY;
            if (!circleInMaterial(cls, execX, execY, probeZ, rOut + margin)) {
                violated = true;
                if (!circleInMaterial(cls, execX, execY, probeZ, margin)) {
                    feasibleR = 0.0;   // even the bare margin circle overhangs
                } else {
                    double lo = 0.0, hi = rOut;
                    for (int i = 0; i < 24; ++i) {
                        const double mid = 0.5 * (lo + hi);
                        if (circleInMaterial(cls, execX, execY, probeZ,
                                             mid + margin))
                            lo = mid;
                        else
                            hi = mid;
                    }
                    feasibleR = lo;
                }
                result.notes.push_back(
                    "measured footprint: largest feasible radius " +
                    std::to_string(feasibleR) + " mm at the entry plane");
            }
        } else {
            const bool fitsX = offX + rOut <= dstFrame.hx - margin;
            const bool fitsY = offY + rOut <= dstFrame.hy - margin;
            if (!fitsX || !fitsY) {
                violated  = true;
                feasibleR = std::max(0.0,
                                     std::min(dstFrame.hx - margin - offX,
                                              dstFrame.hy - margin - offY));
            }
        }
    }

    if (violated && fam == nullptr) {
        const double feasibleOd = 2.0 * feasibleR;
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
    } else if (violated) {
        const double newPcd = 2.0 * feasibleR - widest;
        if (newPcd <= widest) {
            neuter(result, "post-clamp pitch circle " + std::to_string(newPcd) +
                           " mm <= widest member " + std::to_string(widest) +
                           " mm — the skill's validate rejects it");
            spdlog::debug("adapt::transferFeature {}->{}: refused (post-clamp PCD)",
                          srcProduct, dstProduct);
            return result;
        }
        const double chord = newPcd * std::sin(M_PI / count);
        if (chord < widest) {
            neuter(result, "post-clamp member chord " + std::to_string(chord) +
                           " mm < widest member " + std::to_string(widest) +
                           " mm — adjacent cuts would merge");
            spdlog::debug("adapt::transferFeature {}->{}: refused (members merge)",
                          srcProduct, dstProduct);
            return result;
        }
        if (chord - widest < 1.5)
            result.notes.push_back("post-clamp member gap " +
                                   std::to_string(chord - widest) +
                                   " mm is inside the DFM-003 density "
                                   "warning band");
        p["bolt_circle_dia_mm"] = newPcd;
        result.fit_clamped = true;
        result.notes.push_back("fit clamp: pitch circle " + std::to_string(pcd) +
                               " -> " + std::to_string(newPcd) +
                               " mm (member diameters preserved)");
    }

    // 9. Success.
    result.transferred = true;
    spdlog::debug("adapt::transferFeature {}->{}: '{}' transferred (fit_clamped={})",
                  srcProduct, dstProduct, result.step.skill_id, result.fit_clamped);
    return result;
}

}  // namespace koocadcam::adapt
