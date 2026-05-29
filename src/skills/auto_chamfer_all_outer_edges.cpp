// @lat: [[engine/skills#auto_chamfer_all_outer_edges]]

#include "auto_chamfer_all_outer_edges.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::auto_chamfer_all_outer_edges {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Lookup table: ISO 13715 standard chamfer sizes ───────────────────────
namespace {

struct ChamferSpec {
    const char* key;
    double      size_mm;
};

constexpr std::array<ChamferSpec, 4> kChamferTable {{
    { "small", 0.3 },
    { "med",   0.5 },
    { "std",   1.0 },
    { "large", 2.0 },
}};

const ChamferSpec* lookupSize(const std::string& key)
{
    for (const auto& s : kChamferTable)
        if (key == s.key) return &s;
    return nullptr;
}

// Classify an edge as "outer convex" by inspecting its two adjacent
// faces' normals.  We compute the cross product of n1 × n2 and the
// vector from the workpiece centroid to the edge midpoint; if the dot of
// (n1 + n2) with (mid − centroid) is positive, the edge is OUTER convex.
bool isOuterConvexEdge(const Workpiece& wp, const TopoDS_Edge& edge,
                        const gp_Pnt& midPt, const gp_Pnt& centroid)
{
    int facesAdjacent = 0;
    gp_Dir n1, n2;
    bool gotN1 = false, gotN2 = false;
    for (int f = 0; f < wp.faceCount(); ++f) {
        const TopoDS_Face& face = wp.face(f);
        bool found = false;
        for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
            if (exp.Current().IsSame(edge)) { found = true; break; }
        }
        if (!found) continue;
        try {
            if (!gotN1) { n1 = wp.faceNormal(f); gotN1 = true; }
            else if (!gotN2) { n2 = wp.faceNormal(f); gotN2 = true; }
        } catch (...) {}
        ++facesAdjacent;
        if (facesAdjacent >= 2) break;
    }
    if (!gotN1 || !gotN2) return false;

    const gp_Vec out(centroid, midPt);
    const gp_Vec sumN(n1.X() + n2.X(), n1.Y() + n2.Y(), n1.Z() + n2.Z());
    return sumN.Dot(out) > 0.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.chamfer_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "chamfer_size_mm must be > 0");
    }
    if (in.chamfer_size_mm < 0.1) {
        r.add("DFM-CHAMFER-MIN", "error",
              "chamfer_size " + std::to_string(in.chamfer_size_mm) +
              " mm < 0.1 mm (knife-edge per ISO 13715)");
    }
    const ChamferSpec* spec = lookupSize(in.chamfer_size_key);
    if (!spec) {
        r.add("DFM-CHAMFER-SPEC", "error",
              "auto_chamfer_all_outer_edges: unknown chamfer_size_key '" +
              in.chamfer_size_key + "' (supported: small/med/std/large)");
        return r;
    }

    // Min wall rule.
    if (in.chamfer_size_mm > 0.25 * in.min_wall_thick_mm) {
        r.add("DFM-CHAMFER-MAX", "warning",
              "chamfer_size " + std::to_string(in.chamfer_size_mm) +
              " mm > 0.25 × wall (" +
              std::to_string(0.25 * in.min_wall_thick_mm) + ") — breakthrough risk");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "auto_chamfer_all_outer_edges DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // ── Context: workpiece centroid (bbox centre as proxy) ──────────────
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const gp_Pnt centroid(
        (xMin + xMax) / 2.0,
        (yMin + yMax) / 2.0,
        (zMin + zMax) / 2.0);

    // ── Walk every edge, collect outer convex ones ──────────────────────
    BRepFilletAPI_MakeChamfer chamfer(wp.shape());
    int chamferedCount = 0;
    int outerCount = 0;

    for (TopExp_Explorer exp(wp.shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
        BRepAdaptor_Curve curve(e);
        const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
        gp_Pnt midPt;
        try { midPt = curve.Value(mid); } catch (...) { continue; }

        // Z-filter knobs.
        if (!in.include_top_edges    && std::abs(midPt.Z() - zMax) < 1e-3) continue;
        if (!in.include_bottom_edges && std::abs(midPt.Z() - zMin) < 1e-3) continue;

        if (!isOuterConvexEdge(wp, e, midPt, centroid)) continue;
        ++outerCount;
        chamfer.Add(in.chamfer_size_mm, e);
        ++chamferedCount;
    }

    if (chamferedCount == 0) {
        throw SkillError("auto_chamfer_all_outer_edges: no outer convex edges found");
    }

    chamfer.Build();
    if (!chamfer.IsDone()) {
        throw SkillError("auto_chamfer_all_outer_edges: chamfer build failed");
    }
    TopoDS_Shape afterChamfer = chamfer.Shape();

    // ── Sub-feature 2: weld-relief cut at top centre to guarantee a
    //   re-entry point for the toolpath.  Small cylindrical pocket.
    const double rRelief = std::min(in.chamfer_size_mm * 0.5, 0.3);
    const gp_Pnt reliefPt(
        (xMin + xMax) / 2.0,
        (yMin + yMax) / 2.0,
        zMax + 0.05);
    const gp_Ax2 reliefAx(reliefPt, gp_Dir(0, 0, -1));
    const TopoDS_Shape reliefTool = pr::cylinder(reliefAx, rRelief,
                                                  in.chamfer_size_mm + 0.1);
    const TopoDS_Shape newShape = pr::cut(afterChamfer, reliefTool);

    // ── Bookkeeping ─────────────────────────────────────────────────────
    // Approx removed volume per chamfer ≈ 0.5 × c × c × edge_length.  We
    // don't have exact edge lengths handy — assume average 20 mm.
    const double avgEdgeLen = 20.0;
    const double cs = in.chamfer_size_mm;
    const double vChamferEach = 0.5 * cs * cs * avgEdgeLen;
    const double vChamferAll  = vChamferEach * chamferedCount;
    const double vRelief = M_PI * rRelief * rRelief * (in.chamfer_size_mm + 0.1);

    json params = {
        { "chamfer_size_mm",     in.chamfer_size_mm },
        { "chamfer_size_key",    in.chamfer_size_key },
        { "include_top_edges",   in.include_top_edges },
        { "include_bottom_edges", in.include_bottom_edges },
        { "min_wall_thick_mm",   in.min_wall_thick_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  chamferedCount + 1 },
        { "chamfer_size_mm",   in.chamfer_size_mm },
        { "chamfer_size_key",  in.chamfer_size_key },
        { "outer_edge_count",  outerCount },
        { "chamfered_count",   chamferedCount },
        { "standard",          "ISO 13715" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "chamfer_mill";
    tooling.tool_dia_mm       = std::max(2.0, in.chamfer_size_mm * 4.0);
    tooling.tool_length_mm    = 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = vChamferAll + vRelief;
    tooling.est_cycle_time_s  = std::max(2.0, chamferedCount * 2.0);
    tooling.extra = {
        { "chamfer_volume_mm3", vChamferAll },
        { "relief_volume_mm3",  vRelief },
        { "added_volume_mm3",   0.0 },
        { "removed_volume_mm3", vChamferAll + vRelief },
        { "net_delta_mm3",      -(vChamferAll + vRelief) },
        { "standard",           "ISO 13715" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::auto_chamfer_all_outer_edges: count={} size={}",
                  chamferedCount, in.chamfer_size_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    auto emitMetadataReplay = [&]() {
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            RecognizedFeature rf;
            rf.skill_id = kSkillId;
            rf.recovered_params = f.params;
            rf.confidence = 1.0;
            rf.matched_geometry = { { "via", "metadata_replay" } };
            out.push_back(rf);
            return;
        }
    };

    // Geometric heuristic: count small planar faces tilted ~45° from world
    // axes (typical chamfer faces).  When the count exceeds 4, return a
    // candidate.  Estimate the chamfer size from the smallest of these
    // faces' area: area ≈ c × edge_length, so c ≈ sqrt(area/edge_length).
    int chamferLikeFaces = 0;
    double minArea = std::numeric_limits<double>::infinity();
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const gp_Dir n = wp.faceNormal(i);
        double absX = std::abs(n.X()), absY = std::abs(n.Y()), absZ = std::abs(n.Z());
        // A chamfer face has a normal that's a mix of axes (not pure-X/Y/Z).
        int dominant = (absX > 0.9 ? 1 : 0) + (absY > 0.9 ? 1 : 0) + (absZ > 0.9 ? 1 : 0);
        if (dominant > 0) continue;
        double area = 0.0;
        try { area = wp.faceArea(i); } catch (...) { continue; }
        if (area < 0.05 || area > 50.0) continue;
        ++chamferLikeFaces;
        if (area < minArea) minArea = area;
    }

    if (chamferLikeFaces >= 4) {
        // Estimate chamfer size from smallest tilt-face area (assumed
        // edge_length ≈ 20 mm).
        const double estSize = std::clamp(std::sqrt(std::max(0.01, minArea) / 20.0),
                                          0.1, 5.0);
        // Map estimate to lookup key.
        std::string key = "std";
        double bestErr = 1e6;
        for (const auto& s : kChamferTable) {
            const double e = std::abs(s.size_mm - estSize);
            if (e < bestErr) { bestErr = e; key = s.key; }
        }
        json recovered = {
            { "chamfer_size_mm",     estSize },
            { "chamfer_size_key",    key },
            { "include_top_edges",   true },
            { "include_bottom_edges", true },
            { "min_wall_thick_mm",   4.0 },
        };
        json matched = {
            { "chamfer_like_face_count", chamferLikeFaces },
            { "min_chamfer_face_area",   minArea },
            { "via",                     "geometric_primary" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    }

    if (out.empty()) emitMetadataReplay();
    return out;
}

}  // namespace koocadcam::skill::auto_chamfer_all_outer_edges
