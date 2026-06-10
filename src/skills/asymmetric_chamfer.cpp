// @lat: [[engine/skills#asymmetric_chamfer]]

#include "asymmetric_chamfer.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
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

namespace koocadcam::skill::asymmetric_chamfer {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (!(in.distance_a_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "asymmetric_chamfer: distance_a_mm must be > 0");
    }
    if (!(in.distance_b_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "asymmetric_chamfer: distance_b_mm must be > 0");
    }
    if (!(in.edge_z_band_min < in.edge_z_band_max)) {
        r.add("DFM-INPUT", "error",
              "asymmetric_chamfer: edge_z_band_min must be < edge_z_band_max");
    }
    if (!wp.shape().IsNull()) {
        const auto bb = pr::optimalBbox(wp.shape());
        const double minExt = std::min({ bb.dx(), bb.dy(), bb.dz() });
        const double maxD = std::max(in.distance_a_mm, in.distance_b_mm);
        if (minExt > 0.0 && maxD > 0.5 * minExt) {
            r.add("DFM-WALL", "warning",
                  "asymmetric_chamfer: max distance " + std::to_string(maxD)
                  + " mm > half min-extent " + std::to_string(minExt));
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

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
        std::string msg = "asymmetric_chamfer DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Edge → adjacent faces map (built once on the *initial* shape).
    EdgeFaceMap edgeFaces;
    TopExp::MapShapesAndAncestors(wp.shape(), TopAbs_EDGE, TopAbs_FACE, edgeFaces);

    // Collect edges in band along with one adjacent face each.
    struct EdgeFace { TopoDS_Edge edge; TopoDS_Face face; };
    std::vector<EdgeFace> selected;
    for (TopExp_Explorer e(wp.shape(), TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
        double midZ = 0.0;
        if (!edgeBboxMidZ(edge, midZ)) continue;
        if (midZ < in.edge_z_band_min || midZ > in.edge_z_band_max) continue;

        if (!edgeFaces.Contains(edge)) continue;
        const auto& adj = edgeFaces.FindFromKey(edge);
        if (adj.IsEmpty()) continue;
        const TopoDS_Face face = TopoDS::Face(adj.First());
        selected.push_back({ edge, face });
    }
    if (selected.empty()) {
        throw SkillError("asymmetric_chamfer: no edges in Z band ["
                         + std::to_string(in.edge_z_band_min) + ", "
                         + std::to_string(in.edge_z_band_max) + "]");
    }

    // Try a single combined Build first; if it fails, fall back to per-edge.
    int chamfered = 0;
    int skipped   = 0;
    TopoDS_Shape current = wp.shape();
    bool combinedOk = false;
    try {
        BRepFilletAPI_MakeChamfer cb(current);
        for (const auto& ef : selected) {
            cb.Add(in.distance_a_mm, in.distance_b_mm, ef.edge, ef.face);
        }
        cb.Build();
        if (cb.IsDone()) {
            const TopoDS_Shape next = cb.Shape();
            if (!next.IsNull()) {
                current     = next;
                chamfered   = static_cast<int>(selected.size());
                combinedOk  = true;
            }
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("asymmetric_chamfer: combined Build threw — {}", ex.what());
    } catch (...) { /* fall through */ }

    if (!combinedOk) {
        chamfered = 0;
        current   = wp.shape();
        for (const auto& ef : selected) {
            try {
                BRepFilletAPI_MakeChamfer cb(current);
                cb.Add(in.distance_a_mm, in.distance_b_mm, ef.edge, ef.face);
                cb.Build();
                if (!cb.IsDone()) { ++skipped; continue; }
                const TopoDS_Shape next = cb.Shape();
                if (next.IsNull()) { ++skipped; continue; }
                current = next;
                ++chamfered;
            } catch (const Standard_Failure& ex) {
                spdlog::debug("asymmetric_chamfer: edge skipped — {}", ex.what());
                ++skipped;
            } catch (...) { ++skipped; }
        }
    }

    if (chamfered == 0) {
        throw SkillError("asymmetric_chamfer: OCCT failed on every selected edge");
    }

    double volBefore = 0.0, volAfter = 0.0;
    {
        GProp_GProps gp;
        BRepGProp::VolumeProperties(wp.shape(), gp);
        volBefore = gp.Mass();
        BRepGProp::VolumeProperties(current, gp);
        volAfter = gp.Mass();
    }

    // Count distinct planar chamfer faces in the result (post-build).  We
    // approximate by counting all planar faces minus the planar faces of the
    // original.  This is a coarse number for the signature.
    int planarBefore = 0;
    for (TopExp_Explorer fe(wp.shape(), TopAbs_FACE); fe.More(); fe.Next()) {
        BRepAdaptor_Surface s(TopoDS::Face(fe.Current()));
        if (s.GetType() == GeomAbs_Plane) ++planarBefore;
    }
    int planarAfter = 0;
    for (TopExp_Explorer fe(current, TopAbs_FACE); fe.More(); fe.Next()) {
        BRepAdaptor_Surface s(TopoDS::Face(fe.Current()));
        if (s.GetType() == GeomAbs_Plane) ++planarAfter;
    }
    const int chamferFacesAdded = std::max(0, planarAfter - planarBefore);

    json params = {
        { "edge_z_band_min", in.edge_z_band_min },
        { "edge_z_band_max", in.edge_z_band_max },
        { "distance_a_mm",   in.distance_a_mm },
        { "distance_b_mm",   in.distance_b_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "edge_count_chamfered",       chamfered },
        { "edge_count_skipped",         skipped },
        { "distance_a_mm",              in.distance_a_mm },
        { "distance_b_mm",              in.distance_b_mm },
        { "distinct_chamfer_face_count", chamferFacesAdded },
        { "derived_volume_removed",     volAfter - volBefore },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::max(2.0,
                                     std::max(in.distance_a_mm, in.distance_b_mm) * 4.0);
    tooling.tool_length_mm    = 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = std::abs(volAfter - volBefore);
    tooling.est_cycle_time_s  = std::max(2.0, chamfered * 1.0);
    tooling.extra["asymmetric"] = true;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::asymmetric_chamfer: dA={} dB={} chamfered={} skipped={} "
                  "newPlanarFaces={} dVol={}",
                  in.distance_a_mm, in.distance_b_mm, chamfered, skipped,
                  chamferFacesAdded, volAfter - volBefore);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "edge_z_band_min", f.params.value("edge_z_band_min", 0.0) },
            { "edge_z_band_max", f.params.value("edge_z_band_max", 0.0) },
            { "distance_a_mm",   f.params.value("distance_a_mm",   1.0) },
            { "distance_b_mm",   f.params.value("distance_b_mm",   1.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric: look for small planar faces (potential chamfer faces).  An
    // asymmetric chamfer leaves planar faces whose normals are between the
    // two original face normals at a non-45° angle.  Hard to identify
    // precisely without the original; we report a low-confidence guess.
    int smallPlanar = 0;
    double avgArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            const double a = wp.faceArea(i);
            avgArea += a;
            ++smallPlanar;
        } catch (...) { /* skip */ }
    }
    if (smallPlanar == 0) return out;
    avgArea /= smallPlanar;
    if (smallPlanar < 2) return out;

    RecognizedFeature rf;
    rf.skill_id = kSkillId;
    rf.recovered_params = {
        { "edge_z_band_min", 0.0 },
        { "edge_z_band_max", 0.0 },
        { "distance_a_mm",   1.0 },
        { "distance_b_mm",   0.5 },
    };
    rf.confidence       = 0.30;
    rf.matched_geometry = json{
        { "planar_face_count", smallPlanar },
        { "avg_planar_area",   avgArea },
    };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::asymmetric_chamfer
