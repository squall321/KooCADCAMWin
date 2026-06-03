// @lat: [[engine/skills#multi_radius_fillet_table]]

#include "multi_radius_fillet_table.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::multi_radius_fillet_table {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.edge_specs.empty()) {
        r.add("DFM-INPUT", "error",
              "multi_radius_fillet_table: edge_specs must not be empty");
    }
    double minExt = 0.0;
    if (!wp.shape().IsNull()) {
        const auto bb = pr::optimalBbox(wp.shape());
        minExt = std::min({ bb.dx(), bb.dy(), bb.dz() });
    }
    for (size_t i = 0; i < in.edge_specs.size(); ++i) {
        const auto& s = in.edge_specs[i];
        const std::string idx = std::to_string(i);
        if (!(s.radius_mm > 0.0)) {
            r.add("DFM-INPUT", "error",
                  "multi_radius_fillet_table[" + idx + "]: radius_mm must be > 0");
        }
        if (!(s.z_min < s.z_max)) {
            r.add("DFM-INPUT", "error",
                  "multi_radius_fillet_table[" + idx + "]: z_min must be < z_max");
        }
        if (minExt > 0.0 && s.radius_mm > 0.5 * minExt) {
            r.add("DFM-WALL", "warning",
                  "multi_radius_fillet_table[" + idx + "]: radius "
                  + std::to_string(s.radius_mm)
                  + " > half min-extent " + std::to_string(minExt));
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

bool edgeBboxMidZ(const TopoDS_Edge& edge, double& outZ)
{
    try {
        BRepAdaptor_Curve curve(edge);
        const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
        outZ = curve.Value(mid).Z();
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "multi_radius_fillet_table DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // (radius, edge) work-items.
    struct Work { double radius; TopoDS_Edge edge; };
    std::vector<Work> work;
    for (TopExp_Explorer e(wp.shape(), TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
        double midZ = 0.0;
        if (!edgeBboxMidZ(edge, midZ)) continue;
        for (const auto& spec : in.edge_specs) {
            if (midZ >= spec.z_min && midZ <= spec.z_max) {
                work.push_back({ spec.radius_mm, edge });
                break;   // first-match wins
            }
        }
    }
    if (work.empty()) {
        throw SkillError("multi_radius_fillet_table: no edges matched any spec band");
    }

    // First try a combined Build with all (radius, edge) pairs.
    int filleted = 0;
    int skipped  = 0;
    TopoDS_Shape current = wp.shape();
    bool combinedOk = false;
    try {
        BRepFilletAPI_MakeFillet fb(current);
        for (const auto& w : work) fb.Add(w.radius, w.edge);
        fb.Build();
        if (fb.IsDone()) {
            const TopoDS_Shape next = fb.Shape();
            if (!next.IsNull()) {
                current     = next;
                filleted    = static_cast<int>(work.size());
                combinedOk  = true;
            }
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("multi_radius_fillet_table: combined Build threw — {}", ex.what());
    } catch (...) { /* fall through */ }

    if (!combinedOk) {
        filleted = 0;
        current  = wp.shape();
        for (const auto& w : work) {
            try {
                BRepFilletAPI_MakeFillet fb(current);
                fb.Add(w.radius, w.edge);
                fb.Build();
                if (!fb.IsDone()) { ++skipped; continue; }
                const TopoDS_Shape next = fb.Shape();
                if (next.IsNull()) { ++skipped; continue; }
                current = next;
                ++filleted;
            } catch (const Standard_Failure& ex) {
                spdlog::debug("multi_radius_fillet_table: edge skipped — {}", ex.what());
                ++skipped;
            } catch (...) { ++skipped; }
        }
    }
    if (filleted == 0) {
        throw SkillError("multi_radius_fillet_table: OCCT failed on every edge");
    }

    double volBefore = 0.0, volAfter = 0.0;
    {
        GProp_GProps gp;
        BRepGProp::VolumeProperties(wp.shape(), gp);
        volBefore = gp.Mass();
        BRepGProp::VolumeProperties(current, gp);
        volAfter = gp.Mass();
    }
    const double dVol = volAfter - volBefore;

    // Collect radius list for signature.
    json radii = json::array();
    json specsJ = json::array();
    for (const auto& s : in.edge_specs) {
        radii.push_back(s.radius_mm);
        specsJ.push_back({ { "z_min", s.z_min },
                           { "z_max", s.z_max },
                           { "radius_mm", s.radius_mm } });
    }

    json params = { { "edge_specs", specsJ } };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "edge_count_filleted",    filleted },
        { "edge_count_skipped",     skipped },
        { "radii",                  radii },
        { "spec_count",             in.edge_specs.size() },
        { "derived_volume_removed", dVol },
    };
    ToolingMeta tooling;
    double maxR = 0.0;
    for (const auto& s : in.edge_specs) maxR = std::max(maxR, s.radius_mm);
    tooling.tool_type         = "ball_mill";
    tooling.tool_dia_mm       = maxR * 2.0;
    tooling.tool_length_mm    = std::max(5.0, maxR * 6.0);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = std::abs(dVol);
    tooling.est_cycle_time_s  = std::max(2.0, filleted * 1.2);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::multi_radius_fillet_table: specs={} filleted={} skipped={} dVol={}",
                  in.edge_specs.size(), filleted, skipped, dVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec;
        rec["edge_specs"] = f.params.value("edge_specs", json::array());
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric: cluster blends by radius; each distinct radius implies one
    // spec entry.  Z-band approximated from face centers.
    struct Blend { double r; double z; };
    std::vector<Blend> blends;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        const auto t = s.GetType();
        double r = 0.0;
        if (t == GeomAbs_Cylinder)   r = s.Cylinder().Radius();
        else if (t == GeomAbs_Torus) r = s.Torus().MinorRadius();
        else                         continue;
        if (r <= 0.0) continue;
        double z = 0.0;
        try { z = wp.faceCenter(i).Z(); } catch (...) { continue; }
        blends.push_back({ r, z });
    }
    if (blends.empty()) return out;

    // Cluster by radius.
    std::vector<bool> used(blends.size(), false);
    json specsJ = json::array();
    for (size_t i = 0; i < blends.size(); ++i) {
        if (used[i]) continue;
        double zLo = blends[i].z, zHi = blends[i].z;
        used[i] = true;
        for (size_t j = i + 1; j < blends.size(); ++j) {
            if (used[j]) continue;
            if (std::abs(blends[j].r - blends[i].r) < 1e-3) {
                zLo = std::min(zLo, blends[j].z);
                zHi = std::max(zHi, blends[j].z);
                used[j] = true;
            }
        }
        specsJ.push_back({
            { "z_min", zLo - 0.5 },
            { "z_max", zHi + 0.5 },
            { "radius_mm", blends[i].r },
        });
    }

    RecognizedFeature rf;
    rf.skill_id = kSkillId;
    rf.recovered_params = { { "edge_specs", specsJ } };
    rf.confidence       = specsJ.size() >= 2 ? 0.55 : 0.35;
    rf.matched_geometry = json{ { "blend_cluster_count", specsJ.size() } };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::multi_radius_fillet_table
