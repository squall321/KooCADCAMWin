// @lat: [[engine/skills#chamfer_edge]]

#include "chamfer_edge.hpp"

#include "Workpiece.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRep_Tool.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace koocadcam::skill::chamfer_edge {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.chamfer_size_mm < 0.1) {
        r.add("DFM-INPUT", "error",
              "chamfer_edge chamfer_size " + std::to_string(in.chamfer_size_mm) +
              " mm < min 0.1 mm");
    }
    if (in.chamfer_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "chamfer_edge chamfer_size must be > 0");
    }
    if (in.angle_deg < 15.0 || in.angle_deg > 75.0) {
        r.add("DFM-011", "error",
              "chamfer_edge angle " + std::to_string(in.angle_deg) +
              "° outside [15°, 75°] — approaches knife-edge");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Same selector→predicate logic as fillet_edge.cpp.  Kept local here so
// each skill is independently understandable.
using EdgePred = std::function<bool(const TopoDS_Edge&, const gp_Pnt&)>;

EdgePred buildPredicate(const Workpiece& wp,
                        const EdgeSelector& sel,
                        int& outResolvedEdgeId)
{
    outResolvedEdgeId = -1;

    return std::visit([&](const auto& s) -> EdgePred {
        using T = std::decay_t<decltype(s)>;

        if constexpr (std::is_same_v<T, EdgesAtZBand>) {
            const double z   = s.z_mm;
            const double tol = s.tolerance_mm;
            return [z, tol](const TopoDS_Edge&, const gp_Pnt& mp) {
                return std::abs(mp.Z() - z) < tol;
            };
        }
        else if constexpr (std::is_same_v<T, EdgeDatum>) {
            auto idOpt = wp.resolve(s);
            if (!idOpt) {
                return [](const TopoDS_Edge&, const gp_Pnt&) { return false; };
            }
            outResolvedEdgeId = *idOpt;
            const TopoDS_Edge target = wp.edge(*idOpt);
            return [target](const TopoDS_Edge& e, const gp_Pnt&) {
                return e.IsSame(target);
            };
        }
        else {
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
        std::string msg = "chamfer_edge DFM failed:";
        for (const auto& f : dfm.findings) {
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        }
        throw SkillError(msg);
    }

    int resolvedEdgeId = -1;
    EdgePred pred = buildPredicate(wp, in.edge_selector, resolvedEdgeId);

    // 2) Walk edges, add to chamfer builder
    BRepFilletAPI_MakeChamfer chamfer(wp.shape());
    int selectedCount = 0;
    for (TopExp_Explorer e(wp.shape(), TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
        BRepAdaptor_Curve curve(edge);
        const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
        const gp_Pnt midPt = curve.Value(mid);
        if (!pred(edge, midPt)) continue;
        chamfer.Add(in.chamfer_size_mm, edge);
        ++selectedCount;
    }
    if (selectedCount == 0) {
        throw SkillError("chamfer_edge: edge_selector matched no edges");
    }

    chamfer.Build();
    if (!chamfer.IsDone()) {
        throw SkillError("chamfer_edge: OCCT chamfer build failed");
    }
    const TopoDS_Shape newShape = chamfer.Shape();

    // 3) Signature
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
        { "edge_selector",   selJson },
        { "chamfer_size_mm", in.chamfer_size_mm },
        { "angle_deg",       in.angle_deg },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "bevel_face_count",  selectedCount },
        { "chamfer_size_mm",   in.chamfer_size_mm },
        { "angle_deg",         in.angle_deg },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::max(2.0, in.chamfer_size_mm * 4.0);
    tooling.tool_length_mm    = 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = std::max(2.0, selectedCount * 1.0);
    tooling.extra["chamfer_angle_deg"] = in.angle_deg;

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::chamfer_edge applied: size={} angle={} edges={} faces {}→{}",
                  in.chamfer_size_mm, in.angle_deg, selectedCount,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A chamfer face is planar and lies between two other planar faces that
// were once adjacent across a sharp edge.  Heuristic:
//   1. Build an edge→faces adjacency map.
//   2. For each shared edge whose two adjacent faces are both planar AND
//      have normals that are NOT (anti-)parallel (i.e. an "inner corner"
//      that has already been chamfered), check if a small third planar
//      face shares a vertex with that edge: that's the chamfer face.
//
// For phase-1 we use a simpler classification: a "small" planar face
// (area < a threshold) whose normal is roughly halfway between two larger
// planar faces is treated as a chamfer face.  Confidence is moderate
// because pure planar chamfer recognition is hard without prior topology.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

double angleBetweenDir(const gp_Dir& a, const gp_Dir& b)
{
    const double dot = std::clamp(
        a.X()*b.X() + a.Y()*b.Y() + a.Z()*b.Z(), -1.0, 1.0);
    return std::acos(dot) * 180.0 / M_PI;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    const auto edgeFaces = buildEdgeFaceMap(shape);

    // Pre-compute face metadata.
    struct FaceMeta {
        gp_Dir normal{ 0.0, 0.0, 1.0 };  // explicit default so /WX doesn't bite
        double area   = 0.0;
        bool   planar = false;
    };
    std::vector<FaceMeta> meta(wp.faceCount());
    for (int i = 0; i < wp.faceCount(); ++i) {
        meta[i].planar = wp.isFacePlanar(i);
        meta[i].area   = meta[i].planar ? wp.faceArea(i) : 0.0;
        if (meta[i].planar) {
            try { meta[i].normal = wp.faceNormal(i); }
            catch (...) { meta[i].planar = false; }
        }
    }

    // Build face-index → TopoDS_Face map (for matching against edgeFaces values).
    auto faceIndex = [&](const TopoDS_Face& f) -> int {
        for (int i = 0; i < wp.faceCount(); ++i)
            if (wp.face(i).IsSame(f)) return i;
        return -1;
    };

    std::vector<int> chamferCandidates;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!meta[i].planar) continue;
        // "Small" relative to mean face area.
        const TopoDS_Face& f = wp.face(i);

        // Count neighbours via shared edges; if at least two neighbours are
        // larger planar faces with the chamfer face's normal between theirs,
        // accept i as a chamfer face.
        std::vector<int> neighborPlanarFaces;
        for (TopExp_Explorer e(f, TopAbs_EDGE); e.More(); e.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
            if (!edgeFaces.Contains(edge)) continue;
            const auto& adj = edgeFaces.FindFromKey(edge);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(f)) continue;
                const int idx = faceIndex(af);
                if (idx < 0 || !meta[idx].planar) continue;
                if (std::find(neighborPlanarFaces.begin(), neighborPlanarFaces.end(), idx)
                    == neighborPlanarFaces.end())
                    neighborPlanarFaces.push_back(idx);
            }
        }
        if (neighborPlanarFaces.size() < 2) continue;

        // Check if there's a pair of neighbours whose normals bracket meta[i]'s.
        bool isChamfer = false;
        for (size_t a = 0; a < neighborPlanarFaces.size() && !isChamfer; ++a) {
            for (size_t b = a + 1; b < neighborPlanarFaces.size() && !isChamfer; ++b) {
                const int na = neighborPlanarFaces[a];
                const int nb = neighborPlanarFaces[b];
                // Skip if both neighbours are smaller than us — chamfer face is
                // expected to be the smaller one in its locality.
                if (meta[na].area < meta[i].area && meta[nb].area < meta[i].area)
                    continue;
                const double angAB = angleBetweenDir(meta[na].normal, meta[nb].normal);
                if (angAB < 20.0 || angAB > 160.0) continue;  // not a real corner
                const double angIA = angleBetweenDir(meta[i].normal, meta[na].normal);
                const double angIB = angleBetweenDir(meta[i].normal, meta[nb].normal);
                // i's normal between na and nb: angIA + angIB ≈ angAB.
                if (std::abs((angIA + angIB) - angAB) < 5.0) {
                    isChamfer = true;
                }
            }
        }
        if (isChamfer) {
            chamferCandidates.push_back(i);
        }
    }

    if (chamferCandidates.empty()) return out;

    // Group by Z-midpoint (same heuristic as fillet_edge), record one
    // candidate per group.
    std::vector<bool> usedC(chamferCandidates.size(), false);
    for (size_t i = 0; i < chamferCandidates.size(); ++i) {
        if (usedC[i]) continue;
        std::vector<int> cluster{ chamferCandidates[i] };
        const double zRef = wp.faceCenter(chamferCandidates[i]).Z();
        usedC[i] = true;
        for (size_t j = i + 1; j < chamferCandidates.size(); ++j) {
            if (usedC[j]) continue;
            if (std::abs(wp.faceCenter(chamferCandidates[j]).Z() - zRef) < 0.5) {
                cluster.push_back(chamferCandidates[j]);
                usedC[j] = true;
            }
        }

        // Estimate chamfer size from face area: for a face with one straight
        // edge of length L and a chamfer size c at 45°, area ≈ c × L × √2 .
        // We can't separate c and L without more analysis; we report area
        // diagnostic and assume the user-supplied chamfer_size as 0.5 mm
        // typical.  Recovered_size_mm is a coarse estimate.
        double estSize = 0.5;  // sentinel
        // Use the smallest cluster face area to estimate.
        double smallest = 1e30;
        for (int fid : cluster) smallest = std::min(smallest, wp.faceArea(fid));
        if (smallest > 0 && smallest < 1e29) {
            // Assume L ≈ side of stock ≈ 10 mm typical; c ≈ smallest / (10 * √2).
            estSize = std::clamp(smallest / (10.0 * std::sqrt(2.0)), 0.05, 5.0);
        }

        double conf = 0.55;
        if (cluster.size() >= 2) conf = 0.70;
        if (cluster.size() >= 4) conf = 0.85;

        json face_ids = json::array();
        double zMin = 1e30, zMax = -1e30;
        for (int fid : cluster) {
            face_ids.push_back(fid);
            const double z = wp.faceCenter(fid).Z();
            zMin = std::min(zMin, z);
            zMax = std::max(zMax, z);
        }
        json selectorJson = {
            { "kind",         "edges_at_z" },
            { "z_mm",         (zMin + zMax) / 2.0 },
            { "tolerance_mm", std::max((zMax - zMin) + 1e-3, 1e-3) },
        };
        json recovered = {
            { "edge_selector",   selectorJson },
            { "chamfer_size_mm", estSize },
            { "angle_deg",       45.0 },
        };
        json matched = {
            { "bevel_face_ids", face_ids },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::chamfer_edge
