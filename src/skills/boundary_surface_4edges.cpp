// @lat: [[engine/skills#boundary_surface_4edges]]

#include "boundary_surface_4edges.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepFill_Filling.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

#include <GeomAbs_Shape.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::boundary_surface_4edges {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

double polylineLength(const std::vector<gp_Pnt>& pts)
{
    double L = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        L += pts[i].Distance(pts[i - 1]);
    }
    return L;
}

// Build a single TopoDS_Edge as the chained polyline (straight segments).
// BRepFill_Filling.Add(edge, ...) accepts ONE edge per call, so we use the
// straight first-to-last segment of each boundary polyline as the constraint
// edge. For higher fidelity, we add intermediate vertex constraints via
// BRepFill_Filling.Add(gp_Pnt). This keeps the surface bound to the polyline.
TopoDS_Edge straightBoundaryEdge(const std::vector<gp_Pnt>& pts)
{
    if (pts.size() < 2) throw Standard_Failure("boundary needs ≥ 2 points");
    BRepBuilderAPI_MakeEdge em(pts.front(), pts.back());
    if (!em.IsDone()) throw Standard_Failure("boundary edge failed");
    return em.Edge();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    // std::array<...,4> always has size 4 — but each polyline must have ≥ 2 points.
    for (std::size_t i = 0; i < 4; ++i) {
        if (in.boundaries[i].size() < 2) {
            r.add("DFM-INPUT", "error",
                  "boundary_surface_4edges: boundary " + std::to_string(i) +
                  " needs ≥ 2 points (got " +
                  std::to_string(in.boundaries[i].size()) + ")");
        } else if (polylineLength(in.boundaries[i]) <= 1e-9) {
            r.add("DFM-BSURF-LEN", "error",
                  "boundary_surface_4edges: boundary " + std::to_string(i) +
                  " has zero length");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "boundary_surface_4edges DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double totalLen = 0.0;
    for (const auto& b : in.boundaries) totalLen += polylineLength(b);

    TopoDS_Shape resultShape;
    bool fillOk = false;
    try {
        // Long-form ctor for OCCT 8.0 compat (matches blend_surface).
        BRepFill_Filling filler(3, 15, 2, false, 1e-3, 1e-2, 1e-1, 0.1, 8, 9);
        for (std::size_t i = 0; i < 4; ++i) {
            const TopoDS_Edge e = straightBoundaryEdge(in.boundaries[i]);
            filler.Add(e, GeomAbs_C0);
            // Intermediate point constraints for higher fidelity to the polyline.
            for (std::size_t k = 1; k + 1 < in.boundaries[i].size(); ++k) {
                filler.Add(in.boundaries[i][k]);
            }
        }
        filler.Build();
        if (filler.IsDone()) {
            const TopoDS_Face f = filler.Face();
            if (!f.IsNull()) {
                resultShape = f;
                fillOk      = true;
            }
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("boundary_surface_4edges: filling threw — {}", ex.what());
    }

    const double volBefore = volumeOf(wp.shape());

    TopoDS_Shape newShape;
    if (fillOk) {
        // A face has zero volume; fuse onto workpiece as a compound so the
        // surface is carried alongside the solid.
        BRep_Builder builder;
        TopoDS_Compound comp;
        builder.MakeCompound(comp);
        if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
        builder.Add(comp, resultShape);
        newShape = comp;
    } else {
        newShape = wp.shape();
    }

    const double volAfter = volumeOf(newShape);
    const double addedVol = volAfter - volBefore;     // ≈ 0 (surface only)

    json bs = json::array();
    for (const auto& b : in.boundaries) {
        json bb = json::array();
        for (const auto& p : b) bb.push_back({ p.X(), p.Y(), p.Z() });
        bs.push_back(bb);
    }

    json params = {
        { "boundaries", bs },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "section_count",         4 },
        { "path_length_mm",        totalLen },
        { "derived_volume_mm3",    addedVol },
        { "fill_done",             fillOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "boundary_surface";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::boundary_surface_4edges: totalLen={} done={}",
                  totalLen, fillOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.95;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric: presence of ≥ 1 BSpline-like face suggests a boundary
    // surface fill — emit a low-confidence candidate.
    int splineLike = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i) && !wp.isFaceCylinder(i)) ++splineLike;
    }
    if (splineLike >= 1) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = json::object();
        rf.confidence       = 0.4;
        rf.matched_geometry = json{ { "spline_face_count", splineLike } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::boundary_surface_4edges
