// @lat: [[engine/skills#fillet_edge]]

#include "fillet_edge.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Fillets.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Torus.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

namespace koocadcam::skill::fillet_edge {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.radius_mm < 0.2) {
        r.add("DFM-004", "error",
              "fillet_edge radius " + std::to_string(in.radius_mm) +
              " mm < min 0.2 mm");
    }
    if (in.radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "fillet_edge radius must be > 0");
    }
    // DFM-011 (anti-knife edge) requires post-fillet geometry analysis to
    // measure adjacent-face dihedral angles.  Emit info note as a marker;
    // process planner enforces this via a separate pass.
    r.add("DFM-011", "info",
          "fillet_edge: adjacent-face angle check (DFM-011) deferred to "
          "post-build geometric analysis");
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Build an edge-selection predicate from the variant input.  For single
// EdgeDatum we resolve the index up-front and compare TopoDS_Edge identity;
// for Z-band we wrap prim::edgesAtZ.
pr::EdgePredicate buildPredicate(const Workpiece& wp,
                                 const EdgeSelector& sel,
                                 int& outResolvedEdgeId)
{
    outResolvedEdgeId = -1;

    return std::visit([&](const auto& s) -> pr::EdgePredicate {
        using T = std::decay_t<decltype(s)>;

        if constexpr (std::is_same_v<T, EdgesAtZBand>) {
            return pr::edgesAtZ(s.z_mm, s.tolerance_mm);
        }
        else if constexpr (std::is_same_v<T, EdgeDatum>) {
            auto idOpt = wp.resolve(s);
            if (!idOpt) {
                // No match — predicate selects nothing.
                return [](const TopoDS_Edge&, const gp_Pnt&) { return false; };
            }
            outResolvedEdgeId = *idOpt;
            const TopoDS_Edge target = wp.edge(*idOpt);
            return [target](const TopoDS_Edge& e, const gp_Pnt&) {
                return e.IsSame(target);
            };
        }
        else {
            // Exhaustive catch — keeps MSVC C4702 happy under /WX.
            return [](const TopoDS_Edge&, const gp_Pnt&) { return false; };
        }
    }, sel);
}

// Compute the average tangent direction at the mid-parameter of an edge.
// Returns false if the curve has no usable tangent (degenerate edge).
bool edgeMidDirection(const TopoDS_Edge& edge, gp_Dir& outDir)
{
    try {
        BRepAdaptor_Curve curve(edge);
        const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
        gp_Pnt   p;
        gp_Vec   v;
        curve.D1(mid, p, v);
        if (v.Magnitude() < 1e-9) return false;
        outDir = gp_Dir(v);
        return true;
    } catch (...) {
        return false;
    }
}

// Filter an existing predicate by an `edge_filter_dir` keyword.
//   "horizontal" : keep edges whose tangent is roughly parallel to the XY
//                  plane (|Z component of unit tangent| < 0.2)
//   "vertical"   : keep edges whose tangent is roughly along Z
//                  (|Z component of unit tangent| > 0.8)
//   ""/"all"     : pass-through
pr::EdgePredicate withDirectionFilter(pr::EdgePredicate base,
                                      const std::string& dir)
{
    if (dir.empty() || dir == "all") return base;
    const bool wantHorizontal = (dir == "horizontal");
    const bool wantVertical   = (dir == "vertical");
    if (!wantHorizontal && !wantVertical) return base;

    return [base, wantHorizontal](const TopoDS_Edge& e, const gp_Pnt& mp) -> bool {
        if (!base(e, mp)) return false;
        gp_Dir d;
        if (!edgeMidDirection(e, d)) return false;
        const double zComp = std::abs(d.Z());
        if (wantHorizontal) return zComp < 0.2;
        return zComp > 0.8;   // wantVertical
    };
}

// Build a fallback (coarser) predicate when the primary one matches zero
// edges.  The fallback ignores face identities entirely and selects every
// edge whose mid-Z falls within a configured band.
//
//   - If `z_min_mm` / `z_max_mm` are provided, the band is [z_min, z_max].
//   - Otherwise, if the primary selector was an EdgesAtZBand, the band is
//     widened around its z_mm by a factor of 100×tolerance (typically
//     1e-3 mm → 0.1 mm, generous enough to absorb Boolean re-fingerprint
//     drift while staying away from neighbouring edge clusters).
//   - Otherwise, returns std::nullopt — no coarser retry possible.
//
// Returned predicate is further refined by `edge_filter_dir` (if set).
std::optional<pr::EdgePredicate> buildFallbackPredicate(const Input& in,
                                                        double& outBandZMin,
                                                        double& outBandZMax)
{
    double zLo = 0.0, zHi = 0.0;
    bool haveBand = false;

    if (in.z_min_mm.has_value() && in.z_max_mm.has_value()) {
        zLo = *in.z_min_mm;
        zHi = *in.z_max_mm;
        if (zHi < zLo) std::swap(zLo, zHi);
        haveBand = true;
    } else if (in.z_min_mm.has_value() || in.z_max_mm.has_value()) {
        // Only one endpoint specified — pair it with the primary z if it was
        // a band.  Otherwise fall through.
        if (const auto* band = std::get_if<EdgesAtZBand>(&in.edge_selector)) {
            const double z0 = band->z_mm;
            zLo = std::min(in.z_min_mm.value_or(z0), in.z_max_mm.value_or(z0));
            zHi = std::max(in.z_min_mm.value_or(z0), in.z_max_mm.value_or(z0));
            haveBand = true;
        }
    } else if (const auto* band = std::get_if<EdgesAtZBand>(&in.edge_selector)) {
        // Widen tolerance ×100 (≥ 0.1 mm) around z_mm.
        const double widened = std::max(band->tolerance_mm * 100.0, 0.1);
        zLo = band->z_mm - widened;
        zHi = band->z_mm + widened;
        haveBand = true;
    }

    if (!haveBand) return std::nullopt;

    outBandZMin = zLo;
    outBandZMax = zHi;

    pr::EdgePredicate band = [zLo, zHi](const TopoDS_Edge&, const gp_Pnt& mp) {
        return (mp.Z() >= zLo) && (mp.Z() <= zHi);
    };
    return withDirectionFilter(std::move(band), in.edge_filter_dir);
}

// Count how many edges of `shape` satisfy `pred`.  Pure TopExp_Explorer
// re-enumeration — no deprecated TopTools_* helpers, no caching.
int countMatchingEdges(const TopoDS_Shape& shape, const pr::EdgePredicate& pred)
{
    int n = 0;
    for (TopExp_Explorer e(shape, TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
        BRepAdaptor_Curve curve(edge);
        const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
        const gp_Pnt midPt = curve.Value(mid);
        if (pred(edge, midPt)) ++n;
    }
    return n;
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fillet_edge DFM failed:";
        for (const auto& f : dfm.findings) {
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        }
        throw SkillError(msg);
    }

    // 2) Build primary predicate + count selected edges
    int resolvedEdgeId = -1;
    pr::EdgePredicate primary = buildPredicate(wp, in.edge_selector, resolvedEdgeId);

    int selectedCount = countMatchingEdges(wp.shape(), primary);

    // 3) Fallback path — when preceding Boolean ops have re-fingerprinted the
    //    topology (TopoDS_Edge identities and face_ids change), a narrow
    //    selector can match zero surviving edges.  Retry with a coarser
    //    Z-band predicate built from optional Input fields.  This is purely
    //    additive: the existing fillet_edge_test cases keep their behaviour
    //    because none of them populate the fallback fields and the primary
    //    selector resolves cleanly on a fresh stock.
    pr::EdgePredicate effective = primary;
    bool usedFallback = false;
    double fallbackZMin = 0.0, fallbackZMax = 0.0;

    if (selectedCount == 0) {
        double zLo = 0.0, zHi = 0.0;
        if (auto fb = buildFallbackPredicate(in, zLo, zHi)) {
            const int fbCount = countMatchingEdges(wp.shape(), *fb);
            if (fbCount > 0) {
                effective       = *fb;
                selectedCount   = fbCount;
                usedFallback    = true;
                fallbackZMin    = zLo;
                fallbackZMax    = zHi;
                spdlog::debug("fillet_edge: primary selector matched 0 edges; "
                              "fallback Z-band [{}, {}] matched {} edge(s)",
                              zLo, zHi, fbCount);
            }
        }
    }

    if (selectedCount == 0) {
        throw SkillError("fillet_edge: edge_selector matched no edges");
    }

    // 4) Apply the fillet (single-pass build)
    const TopoDS_Shape newShape =
        pr::filletEdges(wp.shape(), in.radius_mm, effective);

    // 5) Signature
    json selJson;
    std::visit([&](const auto& s) {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, EdgesAtZBand>) {
            selJson = { { "kind", "edges_at_z" },
                        { "z_mm", s.z_mm },
                        { "tolerance_mm", s.tolerance_mm } };
        } else if constexpr (std::is_same_v<T, EdgeDatum>) {
            selJson = { { "kind", "edge_datum" },
                        { "resolved_edge_id", resolvedEdgeId } };
        } else {
            selJson = { { "kind", "unknown" } };
        }
    }, in.edge_selector);
    if (usedFallback) {
        selJson["fallback"] = {
            { "z_min_mm",        fallbackZMin },
            { "z_max_mm",        fallbackZMax },
            { "edge_filter_dir", in.edge_filter_dir },
        };
    }

    json params = {
        { "edge_selector", selJson },
        { "radius_mm",     in.radius_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "blend_face_count",        selectedCount },
        { "radius_mm",               in.radius_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "ball_mill";
    tooling.tool_dia_mm       = in.radius_mm * 2.0;
    tooling.tool_length_mm    = std::max(5.0, in.radius_mm * 6.0);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = 0.0;   // hard to estimate without edge length
    tooling.est_cycle_time_s  = std::max(2.0, selectedCount * 1.0);

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::fillet_edge applied: radius={} edges={} faces {}→{}",
                  in.radius_mm, selectedCount,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy: every fillet leaves a swept cylindrical (straight-edge fillet)
// or toroidal (circular-edge fillet) face whose minor radius equals the
// fillet radius.  We scan all faces, group those that look like blends,
// and cluster by radius — large groups of consistent-radius blends are
// "the rim was filleted" (high confidence); singletons are "an edge was
// filleted" (lower confidence).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double radTol = 1e-3;

    struct Blend { int face_id; double radius; double mid_z; std::string kind; };
    std::vector<Blend> blends;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        const TopoDS_Face& f = wp.face(fIdx);
        BRepAdaptor_Surface surf(f);
        const auto t = surf.GetType();

        double r = 0.0;
        std::string kind;
        if (t == GeomAbs_Cylinder) {
            r = surf.Cylinder().Radius();
            kind = "cylindrical";
        } else if (t == GeomAbs_Torus) {
            r = surf.Torus().MinorRadius();
            kind = "toroidal";
        } else {
            continue;
        }
        // A fillet blend face is small (its "length" along the edge is the
        // edge length, but its sweep arc is just (π/2) × r typically).  We
        // do not gate on area here — recognize is best-effort.
        try {
            const gp_Pnt c = wp.faceCenter(fIdx);
            blends.push_back({ fIdx, r, c.Z(), kind });
        } catch (...) { /* skip degenerate */ }
    }
    if (blends.empty()) return out;

    // Cluster by radius (within radTol).
    std::vector<bool> used(blends.size(), false);
    for (size_t i = 0; i < blends.size(); ++i) {
        if (used[i]) continue;
        std::vector<size_t> cluster{ i };
        used[i] = true;
        for (size_t j = i + 1; j < blends.size(); ++j) {
            if (used[j]) continue;
            if (std::abs(blends[j].radius - blends[i].radius) < radTol) {
                cluster.push_back(j);
                used[j] = true;
            }
        }
        const double r = blends[i].radius;

        // Confidence heuristic:
        //   1 blend  → 0.55 (could be design surface)
        //   2-3      → 0.75
        //   ≥ 4      → 0.90 (clearly a rim-fillet sweep)
        double conf = 0.55;
        if (cluster.size() >= 2) conf = 0.75;
        if (cluster.size() >= 4) conf = 0.90;

        // Recover the "Z-band" if all faces in the cluster share an
        // approximate Z midpoint.
        double zMin = 1e30, zMax = -1e30;
        json face_ids = json::array();
        for (auto k : cluster) {
            zMin = std::min(zMin, blends[k].mid_z);
            zMax = std::max(zMax, blends[k].mid_z);
            face_ids.push_back(blends[k].face_id);
        }
        const double zSpan = zMax - zMin;
        json selectorJson;
        if (zSpan < 0.5 && cluster.size() >= 2) {
            selectorJson = {
                { "kind",         "edges_at_z" },
                { "z_mm",         (zMin + zMax) / 2.0 },
                { "tolerance_mm", std::max(zSpan + 1e-3, 1e-3) },
            };
        } else {
            selectorJson = {
                { "kind",             "edge_datum" },
                { "resolved_edge_id", -1 },  // we don't recover the exact edge ID
            };
        }

        json recovered = {
            { "edge_selector", selectorJson },
            { "radius_mm",     r },
        };
        json matched = {
            { "blend_face_ids", face_ids },
            { "kinds_first",    blends[i].kind },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::fillet_edge
