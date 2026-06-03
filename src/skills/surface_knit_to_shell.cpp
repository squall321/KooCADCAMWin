// @lat: [[engine/skills#surface_knit_to_shell]]

#include "surface_knit_to_shell.hpp"

#include "Workpiece.hpp"

#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <set>

namespace koocadcam::skill::surface_knit_to_shell {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_ids.size() < 3) {
        r.add("DFM-FACE-COUNT", "error",
              "surface_knit_to_shell: requires at least 3 faces (got " +
              std::to_string(in.face_ids.size()) + ")");
    }
    if (!(in.tolerance_mm > 0.0)) {
        r.add("DFM-TOLERANCE", "error",
              "surface_knit_to_shell: tolerance_mm must be > 0 (got " +
              std::to_string(in.tolerance_mm) + ")");
    }
    // Check all face ids are in range.
    const int nFaces = wp.faceCount();
    std::set<int> seen;
    for (int id : in.face_ids) {
        if (id < 0 || id >= nFaces) {
            r.add("DFM-INPUT", "error",
                  "surface_knit_to_shell: face_id " + std::to_string(id) +
                  " out of range [0," + std::to_string(nFaces) + ")");
        }
        if (!seen.insert(id).second) {
            r.add("DFM-INPUT", "error",
                  "surface_knit_to_shell: duplicate face_id " +
                  std::to_string(id));
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "surface_knit_to_shell DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Run BRepBuilderAPI_Sewing on the selected faces.
    TopoDS_Shape sewnShape;
    bool sewnOk = false;
    double totalArea = 0.0;
    try {
        BRepBuilderAPI_Sewing sewer(in.tolerance_mm);
        for (int id : in.face_ids) {
            const TopoDS_Face& f = wp.face(id);
            sewer.Add(f);
            GProp_GProps gp;
            BRepGProp::SurfaceProperties(f, gp);
            totalArea += gp.Mass();
        }
        sewer.Perform();
        sewnShape = sewer.SewedShape();
        sewnOk = !sewnShape.IsNull();
    } catch (const Standard_Failure& ex) {
        spdlog::debug("surface_knit_to_shell: sewing threw — {}", ex.what());
    }

    // Compose: keep original solid as primary, add Shell as a sub-shape.
    TopoDS_Shape newShape;
    BRep_Builder builder;
    TopoDS_Compound comp;
    builder.MakeCompound(comp);
    if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
    if (sewnOk) builder.Add(comp, sewnShape);
    newShape = comp;

    json params = {
        { "face_ids",     in.face_ids },
        { "tolerance_mm", in.tolerance_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "source_face_count",    static_cast<int>(in.face_ids.size()) },
        { "derived_area",         totalArea },
        { "tolerance_mm",         in.tolerance_mm },
        { "sewing_done",          sewnOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "surface_feature";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::surface_knit_to_shell: faces={} tol={} area={} ok={}",
                  in.face_ids.size(), in.tolerance_mm, totalArea, sewnOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: detect a TopoDS_Shell sub-shape inside a compound
// workpiece (the sewn Shell artifact left by `apply`).  History replay
// gives confidence 1.0.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "face_ids",     f.params.value("face_ids",
                                            std::vector<int>{}) },
            { "tolerance_mm", f.params.value("tolerance_mm", 1e-3) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback — look for any explicit Shell sub-shape.
    if (wp.shape().IsNull()) return out;
    int shellCount = 0;
    for (TopExp_Explorer e(wp.shape(), TopAbs_SHELL, TopAbs_SOLID);
         e.More(); e.Next()) {
        ++shellCount;
    }
    if (shellCount > 0) {
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = {
            { "face_ids",     std::vector<int>{} },
            { "tolerance_mm", 1e-3 },
        };
        rf.confidence       = 0.55;
        rf.matched_geometry = json{
            { "source", "geometry" },
            { "free_shell_count", shellCount },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::surface_knit_to_shell
