// @lat: [[engine/skills#loft_surface]]

#include "loft_surface.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRep_Builder.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::loft_surface {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

namespace {

// Count distinct (non-coincident-with-neighbour) vertices in a section.
int distinctVertexCount(const std::vector<gp_Pnt>& pts)
{
    if (pts.empty()) return 0;
    int n = 1;
    for (size_t i = 1; i < pts.size(); ++i) {
        if (pts[i].Distance(pts[i - 1]) > 1e-6) ++n;
    }
    // If the last unique vertex coincides with the first, that's the
    // explicit closure marker — subtract one.
    if (n >= 2 && pts.front().Distance(pts.back()) < 1e-6) {
        --n;
    }
    return n;
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.sections.size() < 2) {
        r.add("DFM-INPUT", "error",
              "loft_surface needs ≥ 2 sections (got " +
              std::to_string(in.sections.size()) + ")");
        return r;
    }
    for (size_t i = 0; i < in.sections.size(); ++i) {
        const int nUnique = distinctVertexCount(in.sections[i]);
        if (nUnique < 3) {
            r.add("DFM-LOFT-CLOSED", "error",
                  "loft_surface section " + std::to_string(i) +
                  " has < 3 distinct vertices — cannot form a closed polygon");
        }
    }
    // Reject coincident consecutive sections.
    for (size_t i = 1; i < in.sections.size(); ++i) {
        const auto& a = in.sections[i - 1];
        const auto& b = in.sections[i];
        if (!a.empty() && !b.empty() && a.size() == b.size()) {
            bool coincident = true;
            for (size_t k = 0; k < a.size(); ++k) {
                if (a[k].Distance(b[k]) > 1e-6) { coincident = false; break; }
            }
            if (coincident) {
                r.add("DFM-LOFT-COINCIDE", "error",
                      "loft_surface sections " + std::to_string(i - 1) +
                      " and " + std::to_string(i) +
                      " are coincident (no loft possible)");
            }
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

TopoDS_Wire buildClosedPolyWire(const std::vector<gp_Pnt>& pts)
{
    // Skip a trailing duplicate-of-first vertex if present.
    size_t end = pts.size();
    if (end >= 2 && pts.front().Distance(pts.back()) < 1e-6) {
        --end;
    }
    if (end < 3) {
        throw Standard_Failure("loft_surface: section has < 3 distinct vertices");
    }

    BRepBuilderAPI_MakeWire wireMaker;
    for (size_t i = 0; i < end; ++i) {
        const gp_Pnt& p1 = pts[i];
        const gp_Pnt& p2 = pts[(i + 1) % end];
        if (p1.Distance(p2) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(p1, p2);
        if (!em.IsDone()) throw Standard_Failure("loft_surface: edge build failed");
        wireMaker.Add(em.Edge());
    }
    if (!wireMaker.IsDone()) {
        throw Standard_Failure("loft_surface: wire build failed");
    }
    return wireMaker.Wire();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "loft_surface DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    TopoDS_Shape loftShape;
    bool loftOk = false;
    try {
        // isSolid=true asks ThruSections to cap the ends and produce a
        // closed solid when feasible.
        BRepOffsetAPI_ThruSections through(true, in.ruled, 1.0e-4);
        for (const auto& sec : in.sections) {
            const TopoDS_Wire w = buildClosedPolyWire(sec);
            through.AddWire(w);
        }
        through.Build();
        if (through.IsDone()) {
            loftShape = through.Shape();
            loftOk = !loftShape.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("loft_surface: ThruSections threw — {}", ex.what());
        throw SkillError(std::string("loft_surface: ") + ex.what());
    }

    TopoDS_Shape newShape;
    if (loftOk) {
        try {
            newShape = pr::fuse(wp.shape(), loftShape);
        } catch (const Standard_Failure&) {
            BRep_Builder builder;
            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
            builder.Add(comp, loftShape);
            newShape = comp;
        }
    } else {
        newShape = wp.shape();
    }

    json sectionVertexCounts = json::array();
    for (const auto& sec : in.sections) {
        sectionVertexCounts.push_back(distinctVertexCount(sec));
    }

    json params = {
        { "section_count",          static_cast<int>(in.sections.size()) },
        { "section_vertex_count",   sectionVertexCounts },
        { "ruled",                  in.ruled },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "section_count",          static_cast<int>(in.sections.size()) },
        { "section_vertex_count",   sectionVertexCounts },
        { "ruled",                  in.ruled },
        { "loft_done",              loftOk },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "surface_feature";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::loft_surface applied: {} sections done={}",
                  in.sections.size(), loftOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "section_count",
              f.params.value("section_count", 0) },
            { "section_vertex_count",
              f.params.value("section_vertex_count", json::array()) },
            { "ruled",
              f.params.value("ruled", false) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::loft_surface
