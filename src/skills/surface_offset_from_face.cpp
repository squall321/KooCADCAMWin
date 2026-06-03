// @lat: [[engine/skills#surface_offset_from_face]]

#include "surface_offset_from_face.hpp"

#include "Workpiece.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::surface_offset_from_face {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    auto id = wp.resolve(in.source_face);
    if (!id) {
        r.add("DFM-INPUT", "error",
              "surface_offset_from_face: source_face datum unresolved");
    }
    if (std::abs(in.offset_mm) < 1e-9) {
        r.add("DFM-OFFSET", "error",
              "surface_offset_from_face: offset_mm must be non-zero");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "surface_offset_from_face DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int srcId = *wp.resolve(in.source_face);
    const TopoDS_Face& src = wp.face(srcId);
    const gp_Dir n = wp.faceNormal(srcId);

    // Translation: along source normal by offset_mm.
    gp_Trsf trsf;
    trsf.SetTranslation(gp_Vec(n.X() * in.offset_mm,
                               n.Y() * in.offset_mm,
                               n.Z() * in.offset_mm));

    TopoDS_Shape offsetFaceShape;
    bool offsetOk = false;
    double derivedArea = 0.0;
    try {
        // Transform the source face → translated face on parallel plane.
        BRepBuilderAPI_Transform xform(src, trsf, /*Copy=*/true);
        if (xform.IsDone()) {
            offsetFaceShape = xform.Shape();
            offsetOk = !offsetFaceShape.IsNull();
            if (offsetOk) {
                GProp_GProps gp;
                BRepGProp::SurfaceProperties(offsetFaceShape, gp);
                derivedArea = gp.Mass();
            }
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("surface_offset_from_face: transform threw — {}", ex.what());
    }

    // Workpiece geometry unchanged; offset face attached as compound child.
    TopoDS_Shape newShape;
    if (offsetOk) {
        BRep_Builder builder;
        TopoDS_Compound comp;
        builder.MakeCompound(comp);
        if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
        builder.Add(comp, offsetFaceShape);
        newShape = comp;
    } else {
        newShape = wp.shape();
    }

    json params = {
        { "source_face_id", srcId },
        { "offset_mm",      in.offset_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "source_face_count",   1 },
        { "offset_mm",           in.offset_mm },
        { "derived_area",        derivedArea },
        { "offset_done",         offsetOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "surface_feature";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::surface_offset_from_face: face={} offset={} area={} ok={}",
                  srcId, in.offset_mm, derivedArea, offsetOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "source_face_id", f.params.value("source_face_id", -1) },
            { "offset_mm",      f.params.value("offset_mm",      0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::surface_offset_from_face
