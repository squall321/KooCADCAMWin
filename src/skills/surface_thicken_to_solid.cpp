// @lat: [[engine/skills#surface_thicken_to_solid]]

#include "surface_thicken_to_solid.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::surface_thicken_to_solid {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    auto id = wp.resolve(in.source_face);
    if (!id) {
        r.add("DFM-INPUT", "error",
              "surface_thicken_to_solid: source_face datum unresolved");
    }
    if (!(in.thickness_mm > 0.0)) {
        r.add("DFM-THICKNESS", "error",
              "surface_thicken_to_solid: thickness_mm must be > 0 (got " +
              std::to_string(in.thickness_mm) + ")");
    }
    if (in.direction != "outward" && in.direction != "inward") {
        r.add("DFM-DIRECTION", "error",
              "surface_thicken_to_solid: direction must be 'outward' or 'inward'");
    }
    if (id) {
        // Sanity: thickness must not be larger than the workpiece itself.
        const auto bb = pr::optimalBbox(wp.shape());
        const double maxExt = std::max({ bb.dx(), bb.dy(), bb.dz() });
        if (maxExt > 0.0 && in.thickness_mm > 2.0 * maxExt) {
            r.add("DFM-THICKNESS", "warning",
                  "surface_thicken_to_solid: thickness_mm " +
                  std::to_string(in.thickness_mm) +
                  " exceeds 2× workpiece max extent " +
                  std::to_string(maxExt));
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "surface_thicken_to_solid DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int srcId = *wp.resolve(in.source_face);
    const TopoDS_Face& src = wp.face(srcId);

    // Face normal — outward by default; flip for "inward".
    gp_Dir n = wp.faceNormal(srcId);
    if (in.direction == "inward") n.Reverse();

    // Build extrusion vector and prism.
    const gp_Vec ext(n.X() * in.thickness_mm,
                     n.Y() * in.thickness_mm,
                     n.Z() * in.thickness_mm);

    TopoDS_Shape prismShape;
    bool prismOk = false;
    try {
        BRepPrimAPI_MakePrism prism(src, ext);
        prism.Build();
        if (prism.IsDone()) {
            prismShape = prism.Shape();
            prismOk = !prismShape.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("surface_thicken_to_solid: prism threw — {}", ex.what());
    }

    // Fuse the prism with the workpiece; fall back to compound on failure.
    TopoDS_Shape newShape = wp.shape();
    bool fusedOk = false;
    if (prismOk) {
        try {
            newShape = pr::fuse(wp.shape(), prismShape);
            fusedOk = !newShape.IsNull();
        } catch (const Standard_Failure& ex) {
            spdlog::debug("surface_thicken_to_solid: fuse threw — {}", ex.what());
        }
        if (!fusedOk) {
            BRep_Builder builder;
            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
            builder.Add(comp, prismShape);
            newShape = comp;
        }
    }

    // Derived volume of the thickened prism (for signature).
    double derivedVol = 0.0;
    if (prismOk) {
        GProp_GProps gp;
        BRepGProp::VolumeProperties(prismShape, gp);
        derivedVol = gp.Mass();
    }

    const double signedThk = (in.direction == "inward")
                                ? -in.thickness_mm : in.thickness_mm;

    json params = {
        { "source_face_id", srcId },
        { "thickness_mm",   in.thickness_mm },
        { "direction",      in.direction },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "source_face_count",   1 },
        { "thickness_mm",        signedThk },
        { "derived_volume",      derivedVol },
        { "fused",               fusedOk },
        { "prism_done",          prismOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "surface_feature";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::surface_thicken_to_solid: face={} thickness={} dir={} vol={}",
                  srcId, in.thickness_mm, in.direction, derivedVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: a thin-walled solid where (max bbox extent) /
// (min bbox extent) is large suggests a thickened skin.  History replay is
// the high-confidence path.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "source_face_id", f.params.value("source_face_id", -1) },
            { "thickness_mm",   f.params.value("thickness_mm",  0.0) },
            { "direction",      f.params.value("direction",
                                              std::string("outward")) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback — detect thin-walled shells.
    if (wp.shape().IsNull()) return out;
    const auto bb = pr::optimalBbox(wp.shape());
    const double minExt = std::min({ bb.dx(), bb.dy(), bb.dz() });
    const double maxExt = std::max({ bb.dx(), bb.dy(), bb.dz() });
    if (minExt > 1e-6 && maxExt / minExt > 8.0) {
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = {
            { "source_face_id", -1 },
            { "thickness_mm",   minExt },
            { "direction",      "outward" },
        };
        rf.confidence       = 0.35;
        rf.matched_geometry = json{
            { "source", "geometry" },
            { "min_extent", minExt },
            { "max_extent", maxExt },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::surface_thicken_to_solid
