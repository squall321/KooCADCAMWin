// @lat: [[engine/skills#offset_surface]]

#include "offset_surface.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRep_Builder.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::offset_surface {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    auto id = wp.resolve(in.target_face);
    if (!id) {
        r.add("DFM-INPUT", "error", "offset_surface: target_face datum unresolved");
    }

    if (std::abs(in.offset_mm) < 1e-9) {
        r.add("DFM-INPUT", "error",
              "offset_surface offset_mm must be non-zero");
    }

    // |offset| < 0.5 × min bbox extent — beyond that the offset will
    // collide with the opposite side of the shape and self-intersect.
    const auto bb = pr::optimalBbox(wp.shape());
    const double minExt = std::min({ bb.dx(), bb.dy(), bb.dz() });
    if (minExt > 0.0 && std::abs(in.offset_mm) >= 0.5 * minExt) {
        r.add("DFM-OFFSET-MAG", "error",
              "offset_surface |offset| " + std::to_string(std::abs(in.offset_mm)) +
              " mm must be < 0.5 × shape min-extent (" +
              std::to_string(minExt) + ")");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "offset_surface DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int targetId = *wp.resolve(in.target_face);
    const TopoDS_Face& tgt = wp.face(targetId);

    // Build the offset shape from the single target face.  OCCT's
    // BRepOffsetAPI_MakeOffsetShape works on a shape (face/shell), so we
    // pass `tgt` as the seed and an empty list of "faces to remove".
    TopoDS_Shape offsetShape;
    bool offsetOk = false;
    try {
        BRepOffsetAPI_MakeOffsetShape offsetter;
        offsetter.PerformBySimple(tgt, in.offset_mm);
        if (offsetter.IsDone()) {
            offsetShape = offsetter.Shape();
            offsetOk = !offsetShape.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("offset_surface: offset threw — {}", ex.what());
    }

    // Compose the final shape: original workpiece + offset face (compound).
    TopoDS_Shape newShape;
    if (offsetOk) {
        BRep_Builder builder;
        TopoDS_Compound comp;
        builder.MakeCompound(comp);
        if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
        builder.Add(comp, offsetShape);
        newShape = comp;
    } else {
        // Offset failed: still preserve the workpiece so callers can chain.
        newShape = wp.shape();
    }

    json params = {
        { "target_face_id", targetId },
        { "offset_mm",      in.offset_mm },
    };
    json pattern = {
        { "kind",            kSkillId },
        { "target_face_id",  targetId },
        { "offset_mm",       in.offset_mm },
        { "offset_done",     offsetOk },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "surface_feature";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::offset_surface applied: target_id={} offset={} done={}",
                  targetId, in.offset_mm, offsetOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "target_face_id", f.params.value("target_face_id", -1) },
            { "offset_mm",      f.params.value("offset_mm",      0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::offset_surface
