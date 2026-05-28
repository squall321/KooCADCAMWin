// @lat: [[engine/skills#datum_face_establish]]

#include "datum_face_establish.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::datum_face_establish {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": depth_mm must be > 0 (got " +
              std::to_string(in.depth_mm) + ")");
    }
    if (in.datum_class != "A" && in.datum_class != "B" && in.datum_class != "C") {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": datum_class must be A, B or C (got '" +
              in.datum_class + "')");
    }
    if (in.required_flatness_um <= 0.0) {
        r.add("DFM-FLATNESS", "error",
              std::string(kSkillId) + ": required_flatness_um must be > 0 (got " +
              std::to_string(in.required_flatness_um) + ")");
    } else if (in.required_flatness_um > 50.0) {
        r.add("DF-FLAT-LOOSE", "warning",
              std::string(kSkillId) + ": required_flatness_um " +
              std::to_string(in.required_flatness_um) +
              " µm is loose — typical face-mill achieves 5–25 µm");
    }
    if (in.depth_mm >= 5.0) {
        r.add("DF-DEPTH-DEEP", "warning",
              std::string(kSkillId) + ": depth_mm " +
              std::to_string(in.depth_mm) + " mm ≥ 5 mm — datum cleanup is "
              "normally a shallow skim");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 1) Resolve datum face
    auto entryId = wp.resolve(in.datum_face);
    if (!entryId) throw SkillError(std::string(kSkillId) + ": datum_face unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError(std::string(kSkillId) + ": datum_face must be planar");

    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    // 2) Skim-box geometry — same construction as face_milling.
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double bboxDiag = std::sqrt(bb.dx() * bb.dx()
                                    + bb.dy() * bb.dy()
                                    + bb.dz() * bb.dz());
    const double pad = bboxDiag * 1.1;

    gp_Dir refDir = gp::DX();
    if (std::abs(outNorm.Dot(refDir)) > 0.9) refDir = gp::DY();
    {
        gp_Vec rv(refDir);
        const gp_Vec nv(outNorm);
        rv -= nv * rv.Dot(nv);
        if (rv.Magnitude() < 1e-9) rv = gp_Vec(gp::DY());
        rv.Normalize();
        refDir = gp_Dir(rv);
    }

    const gp_Dir cutMain(-outNorm.X(), -outNorm.Y(), -outNorm.Z());
    const gp_Vec cutMainVec(cutMain);
    const gp_Vec refVec(refDir);
    const gp_Vec yLocalVec = cutMainVec.Crossed(refVec);
    const gp_Dir yLocal(yLocalVec);

    const double overhang = 0.01;
    const gp_Pnt boxOrigin(
        faceCtr.X() + outNorm.X() * overhang - refDir.X() * pad - yLocal.X() * pad,
        faceCtr.Y() + outNorm.Y() * overhang - refDir.Y() * pad - yLocal.Y() * pad,
        faceCtr.Z() + outNorm.Z() * overhang - refDir.Z() * pad - yLocal.Z() * pad);

    const gp_Ax2 boxAx(boxOrigin, cutMain, refDir);
    const TopoDS_Shape skimBox = pr::box(boxAx,
                                          2.0 * pad,
                                          2.0 * pad,
                                          in.depth_mm + overhang);

    // 3) Cut
    const TopoDS_Shape newShape = pr::cut(wp.shape(), skimBox);

    // 4) Stock removed
    GProp_GProps fp;
    BRepGProp::SurfaceProperties(wp.face(*entryId), fp);
    const double removed = fp.Mass() * in.depth_mm;

    // 5) Signature
    json params = {
        { "datum_face_id",         *entryId },
        { "datum_face_normal",     { outNorm.X(), outNorm.Y(), outNorm.Z() } },
        { "datum_class",           in.datum_class },
        { "required_flatness_um",  in.required_flatness_um },
        { "depth_mm",              in.depth_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "datum_face_id",         *entryId },
        { "datum_face_normal",     { outNorm.X(), outNorm.Y(), outNorm.Z() } },
        { "datum_class",           in.datum_class },
        { "required_flatness_um",  in.required_flatness_um },
        { "depth_mm",              in.depth_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = 25.0;
    tooling.tool_length_mm    = 30.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 5;
    tooling.cutting_speed_sfm = 700.0;
    tooling.feed_per_tooth_mm = 0.05;          // finer pass for flat surface
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = std::max(4.0, fp.Mass() / 2000.0);
    tooling.extra["datum_class"]          = in.datum_class;
    tooling.extra["required_flatness_um"] = in.required_flatness_um;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::datum_face_establish applied: class={} flat={}µm depth={}mm",
                  in.datum_class, in.required_flatness_um, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay only) ───────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::datum_face_establish
