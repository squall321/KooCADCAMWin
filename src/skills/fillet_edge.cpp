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
#include <gp_Torus.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

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

    // 2) Build predicate + count selected edges (signature accounting)
    int resolvedEdgeId = -1;
    pr::EdgePredicate pred = buildPredicate(wp, in.edge_selector, resolvedEdgeId);

    int selectedCount = 0;
    for (TopExp_Explorer e(wp.shape(), TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
        BRepAdaptor_Curve curve(edge);
        const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
        const gp_Pnt midPt = curve.Value(mid);
        if (pred(edge, midPt)) ++selectedCount;
    }
    if (selectedCount == 0) {
        throw SkillError("fillet_edge: edge_selector matched no edges");
    }

    // 3) Apply the fillet (single-pass build)
    const TopoDS_Shape newShape = pr::filletEdges(wp.shape(), in.radius_mm, pred);

    // 4) Signature
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
