// @lat: [[engine/skills#shape_morph_interpolate]]

#include "shape_morph_interpolate.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::shape_morph_interpolate {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── helpers ──────────────────────────────────────────────────────────────

namespace {

int distinctVertexCount(const std::vector<gp_Pnt>& pts)
{
    if (pts.empty()) return 0;
    int n = 1;
    for (size_t i = 1; i < pts.size(); ++i) {
        if (pts[i].Distance(pts[i - 1]) > 1e-6) ++n;
    }
    if (n >= 2 && pts.front().Distance(pts.back()) < 1e-6) --n;
    return n;
}

std::vector<gp_Pnt> lerpSections(const std::vector<gp_Pnt>& A,
                                 const std::vector<gp_Pnt>& B,
                                 double t)
{
    std::vector<gp_Pnt> out;
    out.reserve(A.size());
    for (size_t i = 0; i < A.size(); ++i) {
        const gp_Pnt& a = A[i];
        const gp_Pnt& b = B[i];
        out.emplace_back(
            a.X() + (b.X() - a.X()) * t,
            a.Y() + (b.Y() - a.Y()) * t,
            a.Z() + (b.Z() - a.Z()) * t);
    }
    return out;
}

TopoDS_Wire buildClosedPolyWire(const std::vector<gp_Pnt>& pts)
{
    size_t end = pts.size();
    if (end >= 2 && pts.front().Distance(pts.back()) < 1e-6) --end;
    if (end < 3)
        throw Standard_Failure(
            "shape_morph_interpolate: section has < 3 distinct vertices");

    BRepBuilderAPI_MakeWire wireMaker;
    for (size_t i = 0; i < end; ++i) {
        const gp_Pnt& p1 = pts[i];
        const gp_Pnt& p2 = pts[(i + 1) % end];
        if (p1.Distance(p2) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(p1, p2);
        if (!em.IsDone())
            throw Standard_Failure(
                "shape_morph_interpolate: edge build failed");
        wireMaker.Add(em.Edge());
    }
    if (!wireMaker.IsDone())
        throw Standard_Failure(
            "shape_morph_interpolate: wire build failed");
    return wireMaker.Wire();
}

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.section_a.size() < 3 || in.section_b.size() < 3) {
        r.add("DFM-INPUT", "error",
              "shape_morph_interpolate: each section needs ≥ 3 vertices "
              "(a=" + std::to_string(in.section_a.size()) +
              ", b=" + std::to_string(in.section_b.size()) + ")");
        return r;
    }
    if (in.section_a.size() != in.section_b.size()) {
        r.add("DFM-INPUT", "error",
              "shape_morph_interpolate: section_a and section_b must have "
              "matching vertex counts (vertex-wise lerp)");
    }
    if (distinctVertexCount(in.section_a) < 3 ||
        distinctVertexCount(in.section_b) < 3) {
        r.add("DFM-MORPH-CLOSED", "error",
              "shape_morph_interpolate: each section must have ≥ 3 "
              "distinct vertices forming a closed polygon");
    }
    if (in.t < 0.0 || in.t > 1.0) {
        r.add("DFM-INPUT", "error",
              "shape_morph_interpolate: t " + std::to_string(in.t) +
              " outside [0, 1]");
    }
    if (in.intermediate_sections < 2) {
        r.add("DFM-INPUT", "error",
              "shape_morph_interpolate: intermediate_sections must be ≥ 2");
    }
    return r;
}

// ── synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shape_morph_interpolate DFM failed:";
        for (const auto& f : dfm.findings)
            msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 1) Generate N intermediate sections symmetric around `t`.
    const int N = std::max(2, in.intermediate_sections);
    std::vector<std::vector<gp_Pnt>> sections;
    sections.reserve(static_cast<size_t>(N));
    for (int k = 0; k < N; ++k) {
        // Distribute parameters around `t`; the centre parameter equals `t`.
        // Range [t/2, (1+t)/2] keeps the morph LOCAL to the target slice
        // while still giving ThruSections multiple waypoints to skin.
        const double tk =
            (in.t * 0.5) + (static_cast<double>(k) /
                            static_cast<double>(N - 1)) *
                           ((1.0 + in.t) * 0.5 - in.t * 0.5);
        sections.push_back(lerpSections(in.section_a, in.section_b, tk));
    }

    // 2) ThruSections over the N intermediate wires → morph solid.
    TopoDS_Shape morphSolid;
    try {
        BRepOffsetAPI_ThruSections through(true, false, 1.0e-4);
        for (const auto& sec : sections) {
            through.AddWire(buildClosedPolyWire(sec));
        }
        through.Build();
        if (!through.IsDone())
            throw Standard_Failure(
                "shape_morph_interpolate: ThruSections.Build failed");
        morphSolid = through.Shape();
        if (morphSolid.IsNull())
            throw Standard_Failure(
                "shape_morph_interpolate: ThruSections produced null shape");
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("shape_morph_interpolate: ") + ex.what());
    }

    // 3) Boolean cut — subtract the morph solid from the workpiece.
    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), morphSolid);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("shape_morph_interpolate: cut failed: ") +
                         ex.what());
    }

    const double removed = std::max(0.0, volumeOf(morphSolid));

    // 4) Build signature.
    auto sectionToJson = [](const std::vector<gp_Pnt>& v) {
        json arr = json::array();
        for (const auto& p : v)
            arr.push_back({ p.X(), p.Y(), p.Z() });
        return arr;
    };

    json params = {
        { "section_a",             sectionToJson(in.section_a) },
        { "section_b",             sectionToJson(in.section_b) },
        { "t",                     in.t },
        { "intermediate_sections", N },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           N + 1 },  // N wires + 1 cut
        { "section_vertex_count",       static_cast<int>(in.section_a.size()) },
        { "t",                          in.t },
        { "morph_volume_mm3",           removed },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "morph_compound";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = std::max(1.0, removed / 1000.0);
    tooling.extra = {
        { "operation_chain", {
            "lerp section_a→section_b at t",
            "ThruSections " + std::to_string(N) + " waypoints",
            "Boolean cut from workpiece",
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::shape_morph_interpolate applied: N={} t={} removed={:.2f}",
                  N, in.t, removed);

    return SkillOutput{ wpNew, sig };
}

// ── recognition ──────────────────────────────────────────────────────────
//
// Geometric recognition for a free-form morph is unbounded — the morph
// cavity is by construction not a recognisable analytic primitive.  Slice-1
// strategy: rely on feature-history replay (confidence 1.0).  Geometric
// fallback: when the workpiece has at least one non-planar, non-cylindrical
// face the morph cavity could plausibly live there → return a low-conf
// candidate (0.55).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rp = {
            { "section_a",             f.params.value("section_a", json::array()) },
            { "section_b",             f.params.value("section_b", json::array()) },
            { "t",                     f.params.value("t", 0.5) },
            { "intermediate_sections", f.params.value("intermediate_sections", 3) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rp, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback (low confidence).
    int freeformFaces = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i) && !wp.isFaceCylinder(i)) ++freeformFaces;
    }
    if (freeformFaces >= 1) {
        RecognizedFeature r;
        r.skill_id = kSkillId;
        r.recovered_params = json{
            { "t",                     0.5 },
            { "intermediate_sections", 3 },
        };
        r.confidence = 0.55;
        r.matched_geometry = json{
            { "freeform_face_count", freeformFaces },
            { "note", "geometric morph guess; no history available" },
        };
        out.push_back(std::move(r));
    }
    return out;
}

}  // namespace koocadcam::skill::shape_morph_interpolate
