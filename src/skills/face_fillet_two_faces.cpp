// @lat: [[engine/skills#face_fillet_two_faces]]

#include "face_fillet_two_faces.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::face_fillet_two_faces {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    const int N = wp.faceCount();

    if (in.face_a_id < 0 || in.face_a_id >= N ||
        in.face_b_id < 0 || in.face_b_id >= N)
    {
        r.add("DFM-INPUT", "error",
              "face_fillet_two_faces: face ids must be in [0," + std::to_string(N) + ")");
    }
    if (in.face_a_id == in.face_b_id) {
        r.add("DFM-INPUT", "error",
              "face_fillet_two_faces: face_a_id and face_b_id must differ");
    }
    if (!(in.radius_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "face_fillet_two_faces: radius_mm must be > 0 (got "
              + std::to_string(in.radius_mm) + ")");
    }
    if (!wp.shape().IsNull()) {
        const auto bb = pr::optimalBbox(wp.shape());
        const double minExt = std::min({ bb.dx(), bb.dy(), bb.dz() });
        if (minExt > 0.0 && in.radius_mm > 0.5 * minExt) {
            r.add("DFM-WALL", "warning",
                  "face_fillet_two_faces: radius " + std::to_string(in.radius_mm)
                  + " mm > half min-extent " + std::to_string(minExt));
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "face_fillet_two_faces DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const TopoDS_Face& fA = wp.face(in.face_a_id);
    const TopoDS_Face& fB = wp.face(in.face_b_id);

    // Build edge→ancestor-faces map for the whole shape so we can find the
    // edges that connect A and B.
    EdgeFaceMap edgeFaces;
    TopExp::MapShapesAndAncestors(wp.shape(), TopAbs_EDGE, TopAbs_FACE, edgeFaces);

    std::vector<TopoDS_Edge> shared;
    for (int i = 1; i <= edgeFaces.Extent(); ++i) {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeFaces.FindKey(i));
        const auto& adj = edgeFaces.FindFromIndex(i);
        bool seesA = false, seesB = false;
        for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
            const TopoDS_Face& af = TopoDS::Face(it.Value());
            if (af.IsSame(fA)) seesA = true;
            if (af.IsSame(fB)) seesB = true;
        }
        if (seesA && seesB) shared.push_back(edge);
    }
    if (shared.empty()) {
        throw SkillError("face_fillet_two_faces: face_a_id="
                         + std::to_string(in.face_a_id) + " and face_b_id="
                         + std::to_string(in.face_b_id) + " share no edges");
    }

    int filleted = 0, skipped = 0;
    TopoDS_Shape current = wp.shape();
    for (const auto& edge : shared) {
        try {
            BRepFilletAPI_MakeFillet fb(current);
            fb.Add(in.radius_mm, edge);
            fb.Build();
            if (!fb.IsDone()) { ++skipped; continue; }
            const TopoDS_Shape next = fb.Shape();
            if (next.IsNull()) { ++skipped; continue; }
            current = next;
            ++filleted;
        } catch (const Standard_Failure& ex) {
            spdlog::debug("face_fillet_two_faces: edge skipped — {}", ex.what());
            ++skipped;
        } catch (...) { ++skipped; }
    }
    if (filleted == 0) {
        throw SkillError("face_fillet_two_faces: OCCT failed on every shared edge");
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

    json params = {
        { "face_a_id", in.face_a_id },
        { "face_b_id", in.face_b_id },
        { "radius_mm", in.radius_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "edge_count_filleted",    filleted },
        { "edge_count_skipped",     skipped },
        { "radius_mm",              in.radius_mm },
        { "derived_volume_removed", dVol },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "ball_mill";
    tooling.tool_dia_mm       = in.radius_mm * 2.0;
    tooling.tool_length_mm    = std::max(5.0, in.radius_mm * 6.0);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = std::abs(dVol);
    tooling.est_cycle_time_s  = std::max(2.0, filleted * 1.5);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::face_fillet_two_faces: A={} B={} r={} filleted={}/{}",
                  in.face_a_id, in.face_b_id, in.radius_mm, filleted, skipped);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "face_a_id", f.params.value("face_a_id", -1) },
            { "face_b_id", f.params.value("face_b_id", -1) },
            { "radius_mm", f.params.value("radius_mm", 1.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric scan: cluster cylindrical/toroidal blends by radius — a
    // face-fillet leaves >= 1 blend face whose minor/cyl radius equals the
    // input radius.
    struct Blend { double r; };
    std::vector<Blend> blends;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        const auto t = s.GetType();
        double r = 0.0;
        if (t == GeomAbs_Cylinder)   r = s.Cylinder().Radius();
        else if (t == GeomAbs_Torus) r = s.Torus().MinorRadius();
        else                         continue;
        if (r > 0.0) blends.push_back({ r });
    }
    if (blends.empty()) return out;

    std::vector<bool> used(blends.size(), false);
    for (size_t i = 0; i < blends.size(); ++i) {
        if (used[i]) continue;
        int n = 1; used[i] = true;
        for (size_t j = i + 1; j < blends.size(); ++j) {
            if (used[j]) continue;
            if (std::abs(blends[j].r - blends[i].r) < 1e-3) {
                ++n; used[j] = true;
            }
        }
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = {
            { "face_a_id", -1 },
            { "face_b_id", -1 },
            { "radius_mm", blends[i].r },
        };
        rf.confidence = n >= 2 ? 0.65 : 0.40;
        rf.matched_geometry = json{
            { "blend_cluster_size", n },
            { "radius",             blends[i].r },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::face_fillet_two_faces
