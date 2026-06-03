// @lat: [[engine/skills#full_round_fillet]]

#include "full_round_fillet.hpp"

#include "Workpiece.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
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

namespace koocadcam::skill::full_round_fillet {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    const int N = wp.faceCount();
    auto idOk = [&](int id) { return id >= 0 && id < N; };

    if (!idOk(in.face_a_id) || !idOk(in.face_b_id) || !idOk(in.face_c_id)) {
        r.add("DFM-INPUT", "error",
              "full_round_fillet: face ids must be valid (got " +
              std::to_string(in.face_a_id) + ", " +
              std::to_string(in.face_b_id) + ", " +
              std::to_string(in.face_c_id) + "; N=" + std::to_string(N) + ")");
    }
    if (in.face_a_id == in.face_b_id ||
        in.face_b_id == in.face_c_id ||
        in.face_a_id == in.face_c_id)
    {
        r.add("DFM-INPUT", "error",
              "full_round_fillet: face_a, face_b, face_c must all be distinct");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

// Collect edges shared between two faces.
std::vector<TopoDS_Edge> sharedEdges(const TopoDS_Face& a,
                                     const TopoDS_Face& b)
{
    std::vector<TopoDS_Edge> out;
    for (TopExp_Explorer e1(a, TopAbs_EDGE); e1.More(); e1.Next()) {
        const TopoDS_Edge& ea = TopoDS::Edge(e1.Current());
        for (TopExp_Explorer e2(b, TopAbs_EDGE); e2.More(); e2.Next()) {
            const TopoDS_Edge& eb = TopoDS::Edge(e2.Current());
            if (ea.IsSame(eb)) {
                out.push_back(ea);
                break;
            }
        }
    }
    return out;
}

double edgeLength(const TopoDS_Edge& e)
{
    try {
        BRepAdaptor_Curve c(e);
        const double a = c.FirstParameter();
        const double b = c.LastParameter();
        const int N = 16;
        double L = 0.0;
        gp_Pnt prev = c.Value(a);
        for (int i = 1; i <= N; ++i) {
            const double t = a + (b - a) * static_cast<double>(i) / N;
            const gp_Pnt p = c.Value(t);
            L += prev.Distance(p);
            prev = p;
        }
        return L;
    } catch (...) {
        return 0.0;
    }
}

double approximateRadiusForMiddleFace(const TopoDS_Face& middle)
{
    // Half the average length of the middle face's edges, capped against
    // 1/4 the bbox min extent.  Crude but stable.
    double sumL = 0.0;
    int nE = 0;
    for (TopExp_Explorer e(middle, TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& ed = TopoDS::Edge(e.Current());
        const double L = edgeLength(ed);
        if (L > 0.0) { sumL += L; ++nE; }
    }
    double r = (nE > 0) ? (sumL / nE) * 0.5 : 1.0;

    Bnd_Box bb;
    BRepBndLib::AddOptimal(middle, bb);
    if (!bb.IsVoid()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        bb.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        const double minExt = std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
        if (minExt > 0.0) r = std::min(r, minExt * 0.25);
    }
    if (!(r > 1e-3)) r = 0.5;   // safe minimum
    return r;
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "full_round_fillet DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const TopoDS_Face& fA = wp.face(in.face_a_id);
    const TopoDS_Face& fB = wp.face(in.face_b_id);
    const TopoDS_Face& fC = wp.face(in.face_c_id);

    auto sharedAB = sharedEdges(fA, fB);
    auto sharedBC = sharedEdges(fB, fC);
    if (sharedAB.empty() && sharedBC.empty()) {
        throw SkillError("full_round_fillet: faces A/B/C do not share any edges");
    }

    const double radius = approximateRadiusForMiddleFace(fB);

    int filleted = 0;
    int skipped  = 0;
    TopoDS_Shape current = wp.shape();

    auto applyOneEdge = [&](const TopoDS_Edge& edge) {
        try {
            BRepFilletAPI_MakeFillet fb(current);
            fb.Add(radius, edge);
            fb.Build();
            if (!fb.IsDone()) { ++skipped; return; }
            const TopoDS_Shape next = fb.Shape();
            if (next.IsNull()) { ++skipped; return; }
            current = next;
            ++filleted;
        } catch (const Standard_Failure& ex) {
            spdlog::debug("full_round_fillet: edge skipped — {}", ex.what());
            ++skipped;
        } catch (...) { ++skipped; }
    };

    for (const auto& e : sharedAB) applyOneEdge(e);
    for (const auto& e : sharedBC) applyOneEdge(e);

    if (filleted == 0) {
        throw SkillError("full_round_fillet: OCCT failed on every shared edge ("
                         + std::to_string(skipped) + " skipped)");
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
        { "face_c_id", in.face_c_id },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "edge_count_filleted",    filleted },
        { "edge_count_skipped",     skipped },
        { "approx_radius_mm",       radius },
        { "derived_volume_removed", dVol },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "ball_mill";
    tooling.tool_dia_mm       = radius * 2.0;
    tooling.tool_length_mm    = std::max(5.0, tooling.tool_dia_mm * 3.0);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 = std::abs(dVol);
    tooling.est_cycle_time_s  = std::max(3.0, filleted * 1.5);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::full_round_fillet: faces {}/{}/{} r={} filleted={} skipped={} dVol={}",
                  in.face_a_id, in.face_b_id, in.face_c_id, radius,
                  filleted, skipped, dVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "face_a_id", f.params.value("face_a_id", -1) },
            { "face_b_id", f.params.value("face_b_id", -1) },
            { "face_c_id", f.params.value("face_c_id", -1) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric scan: 3-face full rounds leave a single large cylindrical or
    // toroidal blend tangent to two flat neighbours.  Heuristic confidence.
    int blendCount = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        const auto t = s.GetType();
        if (t == GeomAbs_Cylinder || t == GeomAbs_Torus) ++blendCount;
    }
    if (blendCount > 0) {
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = {
            { "face_a_id", -1 },
            { "face_b_id", -1 },
            { "face_c_id", -1 },
        };
        rf.confidence       = 0.30;   // hard to identify without history
        rf.matched_geometry = json{ { "blend_face_count", blendCount } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::full_round_fillet
