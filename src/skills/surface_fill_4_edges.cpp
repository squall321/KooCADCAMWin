// @lat: [[engine/skills#surface_fill_4_edges]]

#include "surface_fill_4_edges.hpp"

#include "Workpiece.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepFill_Filling.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_Shape.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::surface_fill_4_edges {

using nlohmann::json;

namespace {

double polylineLength(const std::vector<gp_Pnt>& pts)
{
    double len = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
        len += pts[i].Distance(pts[i - 1]);
    }
    return len;
}

// Add every consecutive-point segment of the polyline as a G0 edge
// constraint to the BRepFill_Filling solver.
int addPolylineAsEdges(BRepFill_Filling& filler,
                       const std::vector<gp_Pnt>& pts)
{
    int added = 0;
    for (size_t i = 1; i < pts.size(); ++i) {
        if (pts[i].Distance(pts[i - 1]) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge me(pts[i - 1], pts[i]);
        if (!me.IsDone()) continue;
        const TopoDS_Edge& e = me.Edge();
        filler.Add(e, GeomAbs_C0);
        ++added;
    }
    return added;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& /*wp*/, const Input& in)
{
    DFMReport r;
    // Static-sized array → exactly 4 boundaries by construction; verify each
    // polyline has at least 2 points.
    static_assert(std::tuple_size<decltype(in.boundary_edges)>::value == 4,
                  "surface_fill_4_edges requires exactly 4 boundary edges");

    int validEdges = 0;
    for (size_t i = 0; i < in.boundary_edges.size(); ++i) {
        if (in.boundary_edges[i].size() < 2) {
            r.add("DFM-EDGE-LENGTH", "error",
                  "surface_fill_4_edges: boundary " + std::to_string(i) +
                  " must have ≥ 2 points (got " +
                  std::to_string(in.boundary_edges[i].size()) + ")");
        } else {
            ++validEdges;
        }
    }
    if (validEdges < 4) {
        r.add("DFM-EDGE-COUNT", "error",
              "surface_fill_4_edges: requires 4 valid boundary edges (got " +
              std::to_string(validEdges) + ")");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "surface_fill_4_edges DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Perimeter from polylines (always defined — used by signature even if
    // the filling itself fails).
    double perimeter = 0.0;
    for (const auto& poly : in.boundary_edges)
        perimeter += polylineLength(poly);

    TopoDS_Shape fillFace;
    bool fillOk = false;
    double derivedArea = 0.0;
    try {
        // Tuned defaults from blend_surface — G0 (positional) only.
        BRepFill_Filling filler(3, 15, 2, false, 1e-3, 1e-2, 1e-1, 0.1, 8, 9);
        int totalAdded = 0;
        for (const auto& poly : in.boundary_edges) {
            totalAdded += addPolylineAsEdges(filler, poly);
        }
        if (totalAdded >= 4) {
            filler.Build();
            if (filler.IsDone()) {
                fillFace = filler.Face();
                fillOk = !fillFace.IsNull();
                if (fillOk) {
                    GProp_GProps gp;
                    BRepGProp::SurfaceProperties(fillFace, gp);
                    derivedArea = gp.Mass();
                }
            }
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("surface_fill_4_edges: filling threw — {}", ex.what());
    }

    // Compose: original workpiece + filled face (compound).
    TopoDS_Shape newShape;
    BRep_Builder builder;
    TopoDS_Compound comp;
    builder.MakeCompound(comp);
    if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
    if (fillOk) builder.Add(comp, fillFace);
    newShape = comp;

    // Record corner points of each polyline (start + end) for signature
    // — needed for replay-style recognition.
    json edgesJson = json::array();
    for (const auto& poly : in.boundary_edges) {
        json e = json::object();
        if (!poly.empty()) {
            e["start"] = { poly.front().X(), poly.front().Y(), poly.front().Z() };
            e["end"]   = { poly.back().X(),  poly.back().Y(),  poly.back().Z()  };
            e["points"] = poly.size();
        }
        edgesJson.push_back(e);
    }

    json params = {
        { "boundary_edges", edgesJson },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "source_face_count",   0 },
        { "edge_count",          4 },
        { "derived_area",        derivedArea },
        { "derived_perimeter",   perimeter },
        { "fill_done",           fillOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "surface_feature";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::surface_fill_4_edges: perimeter={} area={} ok={}",
                  perimeter, derivedArea, fillOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "boundary_edges", f.params.value("boundary_edges", json::array()) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::surface_fill_4_edges
