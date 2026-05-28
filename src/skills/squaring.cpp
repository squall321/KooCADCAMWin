// @lat: [[engine/skills#squaring]]

#include "squaring.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <GProp_GProps.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::squaring {

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

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.target_envelope_margin_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": target_envelope_margin_mm must be ≥ 0 (got " +
              std::to_string(in.target_envelope_margin_mm) + ")");
    }
    if (in.target_envelope_margin_mm > 5.0) {
        r.add("SQ-MARGIN-DEEP", "warning",
              std::string(kSkillId) + ": target_envelope_margin_mm " +
              std::to_string(in.target_envelope_margin_mm) +
              " mm > 5 mm — single face-mill pass typically capped at 5 mm");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Compute the bbox-fit envelope.
    //
    //    Design note: we collapse a six-face-mill loop into a single
    //    axis-aligned bbox intersection.  Why a single op?
    //      - All six passes converge on the same envelope (the optimal
    //        axis-aligned bbox inset by the desired margin).  Doing them
    //        sequentially gives the same TopoDS_Shape result.
    //      - BRepAlgoAPI_Common(input, envelopeBox) does it in one solver
    //        call vs six Cuts → fewer BRep history edges and tolerance
    //        accumulation.
    //      - The PER-FACE material removed is still computable from the
    //        bbox geometry alone: each face's lost slab is bounded by the
    //        bbox face on the outside and the trimmed face on the inside.
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double m  = in.target_envelope_margin_mm;

    const double envXmin = bb.xMin + m;
    const double envXmax = bb.xMax - m;
    const double envYmin = bb.yMin + m;
    const double envYmax = bb.yMax - m;
    const double envZmin = bb.zMin + m;
    const double envZmax = bb.zMax - m;

    if (envXmax <= envXmin || envYmax <= envYmin || envZmax <= envZmin) {
        throw SkillError(std::string(kSkillId) +
                         ": target_envelope_margin_mm too large — envelope collapses");
    }

    const double envDx = envXmax - envXmin;
    const double envDy = envYmax - envYmin;
    const double envDz = envZmax - envZmin;

    BRepPrimAPI_MakeBox boxMaker(gp_Pnt(envXmin, envYmin, envZmin),
                                  envDx, envDy, envDz);
    boxMaker.Build();
    if (!boxMaker.IsDone())
        throw SkillError(std::string(kSkillId) + ": envelope box build failed");
    const TopoDS_Shape envelope = boxMaker.Shape();

    // 3) Intersect input with envelope.
    BRepAlgoAPI_Common op(wp.shape(), envelope);
    op.Build();
    if (!op.IsDone())
        throw SkillError(std::string(kSkillId) + ": Common op failed");
    const TopoDS_Shape newShape = op.Shape();

    // 4) Compute per-face material removed.
    //
    //    With an axis-aligned input bbox the rough billet's volume is
    //    bb.dx * bb.dy * bb.dz; the envelope's is envDx * envDy * envDz.
    //    Total material removed = bbox_vol − actual_intersection_vol.
    //    We attribute this to the six faces in proportion to their slab
    //    contribution (face area × inset on that side).
    const double bboxVol     = bb.dx() * bb.dy() * bb.dz();
    const double newVol      = volumeOf(newShape);
    const double totalRemoved = std::max(0.0, bboxVol - newVol);

    // Slab proportions (for an axis-aligned billet these sum to ~bboxVol −
    // envelopeVol; for a rough billet whose corners stick out beyond bbox
    // the per-face values are an indicative distribution).
    auto slab = [&](double inset, double areaOther1, double areaOther2) {
        return inset * areaOther1 * areaOther2;
    };
    const double s_px = slab(m, bb.dy(), bb.dz());
    const double s_nx = slab(m, bb.dy(), bb.dz());
    const double s_py = slab(m, bb.dx(), bb.dz());
    const double s_ny = slab(m, bb.dx(), bb.dz());
    const double s_pz = slab(m, bb.dx(), bb.dy());
    const double s_nz = slab(m, bb.dx(), bb.dy());
    const double slabSum = s_px + s_nx + s_py + s_ny + s_pz + s_nz;

    // If the user asked for zero margin, distribute the rough-corner volume
    // evenly across the six faces.
    auto attr = [&](double slabVal) -> double {
        if (slabSum > 1e-9) return totalRemoved * (slabVal / slabSum);
        return totalRemoved / 6.0;
    };

    json removedPerFace = {
        { "+x", attr(s_px) }, { "-x", attr(s_nx) },
        { "+y", attr(s_py) }, { "-y", attr(s_ny) },
        { "+z", attr(s_pz) }, { "-z", attr(s_nz) },
    };

    // 5) Signature
    json params = {
        { "target_envelope_margin_mm", in.target_envelope_margin_mm },
    };
    json pattern = {
        { "kind",                          kSkillId },
        { "target_envelope_margin_mm",     in.target_envelope_margin_mm },
        { "envelope_size_mm",              { envDx, envDy, envDz } },
        { "envelope_origin_mm",            { envXmin, envYmin, envZmin } },
        { "material_removed_per_face",     removedPerFace },
        { "total_material_removed_mm3",    totalRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = 25.0;
    tooling.tool_length_mm    = 30.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 5;
    tooling.cutting_speed_sfm = 600.0;
    tooling.feed_per_tooth_mm = 0.10;
    tooling.stock_removed_mm3 = totalRemoved;
    // Six face-mill passes; ballpark 8 s per face plus a 4 s buffer.
    tooling.est_cycle_time_s  = std::max(12.0, 6.0 * 8.0 + 4.0);
    tooling.extra["passes"]             = 6;
    tooling.extra["material_per_face"]  = removedPerFace;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::squaring applied: margin={} mm, removed={} mm³",
                  in.target_envelope_margin_mm, totalRemoved);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric: a "squared" workpiece is one whose actual volume closely
// matches its optimal axis-aligned bbox volume.  Without prior state we
// can't recover the margin, so we report 0.0 + confidence 0.40.
//
// Metadata replay: any prior squaring signature is returned at conf 1.0.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay (authoritative).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric fallback.
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double bboxVol = bb.dx() * bb.dy() * bb.dz();
    if (bboxVol < 1e-6) return out;
    const double vol = volumeOf(wp.shape());
    const double ratio = vol / bboxVol;
    if (ratio < 0.95) return out;

    json recovered = { { "target_envelope_margin_mm", 0.0 } };
    json matched   = {
        { "bbox_volume_mm3",    bboxVol },
        { "actual_volume_mm3",  vol },
        { "fill_ratio",         ratio },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.40, matched });
    return out;
}

}  // namespace koocadcam::skill::squaring
