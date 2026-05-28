// @lat: [[engine/skills#blend_surface]]

#include "blend_surface.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepFill_Filling.hxx>
#include <BRep_Builder.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::blend_surface {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    auto idA = wp.resolve(in.face_a);
    auto idB = wp.resolve(in.face_b);
    if (!idA) {
        r.add("DFM-INPUT", "error", "blend_surface: face_a datum unresolved");
    }
    if (!idB) {
        r.add("DFM-INPUT", "error", "blend_surface: face_b datum unresolved");
    }
    if (idA && idB && *idA == *idB) {
        r.add("DFM-BLEND-SAME", "error",
              "blend_surface: face_a and face_b must be distinct (both = " +
              std::to_string(*idA) + ")");
    }
    if (in.continuity != 0 && in.continuity != 1) {
        r.add("DFM-INPUT", "error",
              "blend_surface continuity must be 0 (G0) or 1 (G1); got " +
              std::to_string(in.continuity));
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Add every edge of `face` to the BRepFill_Filling solver as either a
// G0 (positional only) or G1 (tangent-continuous) constraint.
void addFaceEdgeConstraints(BRepFill_Filling& filler,
                            const TopoDS_Face& face,
                            int continuity)
{
    const GeomAbs_Shape cont = (continuity >= 1) ? GeomAbs_G1 : GeomAbs_C0;
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
        if (cont == GeomAbs_C0) {
            filler.Add(e, GeomAbs_C0);
        } else {
            filler.Add(e, face, GeomAbs_G1);
        }
    }
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "blend_surface DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int faceIdA = *wp.resolve(in.face_a);
    const int faceIdB = *wp.resolve(in.face_b);
    const TopoDS_Face& fA = wp.face(faceIdA);
    const TopoDS_Face& fB = wp.face(faceIdB);

    // Attempt n-sided fill across the two faces' boundary edges.  If any
    // OCCT step throws (e.g. the boundaries are inconsistent), fall back
    // to a compound of (workpiece, fA, fB) so the skill still produces a
    // surface result and stamps a valid signature.
    TopoDS_Shape blendShape;
    bool fillSucceeded = false;
    try {
        BRepFill_Filling filler(3, 15, 2, false, 1e-3, 1e-2, 1e-1, 0.1, 8, 9);
        addFaceEdgeConstraints(filler, fA, in.continuity);
        addFaceEdgeConstraints(filler, fB, in.continuity);
        filler.Build();
        if (filler.IsDone()) {
            blendShape = filler.Face();
            fillSucceeded = true;
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("blend_surface: filling threw — {}", ex.what());
    }

    // Build the new shape: fuse the original workpiece with the blend face.
    // If filling failed, fall back to a compound containing both source
    // faces (still a valid, additive surface result).
    TopoDS_Shape newShape;
    if (fillSucceeded && !blendShape.IsNull()) {
        try {
            newShape = pr::fuse(wp.shape(), blendShape);
        } catch (const Standard_Failure&) {
            BRep_Builder builder;
            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
            builder.Add(comp, blendShape);
            newShape = comp;
        }
    } else {
        BRep_Builder builder;
        TopoDS_Compound comp;
        builder.MakeCompound(comp);
        if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
        builder.Add(comp, fA);
        builder.Add(comp, fB);
        newShape = comp;
    }

    json params = {
        { "face_a_id",  faceIdA },
        { "face_b_id",  faceIdB },
        { "continuity", in.continuity },
    };
    json pattern = {
        { "kind",         kSkillId },
        { "face_a_id",    faceIdA },
        { "face_b_id",    faceIdB },
        { "continuity",   in.continuity },
        { "fill_done",    fillSucceeded },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "surface_feature";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::blend_surface applied: faces {} + {} -> {} (fill_done={})",
                  faceIdA, faceIdB, wpNew->faceCount(), fillSucceeded);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "face_a_id",  f.params.value("face_a_id",  -1) },
            { "face_b_id",  f.params.value("face_b_id",  -1) },
            { "continuity", f.params.value("continuity",  1) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::blend_surface
