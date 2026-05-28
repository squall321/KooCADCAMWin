// @lat: [[engine/skills#polish_op]]

#include "polish_op.hpp"

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

namespace koocadcam::skill::polish_op {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// Threshold below which we treat removal as a no-op (within OCCT
// Precision::Confusion() — anything thinner cannot survive a Boolean cut).
// In practice DFM-POLISH-MIN already gates inputs at 0.001 mm, so this path
// is a safety net for edge cases (e.g. callers bypassing DFM with explicit
// SkillOutput composition) rather than a normally-reached branch.
static constexpr double kOcctMinSkim = 1e-6;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.removal_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "polish_op removal_mm must be > 0");
    }
    if (in.removal_mm > 0.0 && in.removal_mm < 0.001) {
        r.add("DFM-POLISH-MIN", "error",
              "polish_op removal_mm " + std::to_string(in.removal_mm) +
              " < 0.001 mm — beyond practical polishing precision");
    }
    if (in.removal_mm > 0.05) {
        r.add("DFM-POLISH-MAX", "error",
              "polish_op removal_mm " + std::to_string(in.removal_mm) +
              " > 0.05 mm — exceeds typical polish stock allowance "
              "(use face_milling for material removal, polish for finish)");
    }
    if (in.target_ra.empty()) {
        r.add("DFM-INPUT", "warning",
              "polish_op target_ra empty — defaulting to ra_0.05");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "polish_op DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("polish_op: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("polish_op: entry_face must be planar");

    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    // Record the face area NOW (the entryId becomes stale after the cut).
    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    const double entryArea = props.Mass();

    const std::string targetRa = in.target_ra.empty() ? "ra_0.05" : in.target_ra;

    TopoDS_Shape newShape = wp.shape();
    bool        geomChanged = false;

    if (in.removal_mm >= kOcctMinSkim) {
        // Real micro-skim — same geometry recipe as face_milling::apply, but
        // with a tiny depth.  We re-derive the cutter without recursing into
        // face_milling so the call stays cheap and self-contained.
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
        const gp_Vec yLocalVec = gp_Vec(cutMain).Crossed(gp_Vec(refDir));
        const gp_Dir yLocal(yLocalVec);

        const double overhang = 0.01;
        const gp_Pnt boxOrigin(
            faceCtr.X() + outNorm.X() * overhang
                        - refDir.X() * pad - yLocal.X() * pad,
            faceCtr.Y() + outNorm.Y() * overhang
                        - refDir.Y() * pad - yLocal.Y() * pad,
            faceCtr.Z() + outNorm.Z() * overhang
                        - refDir.Z() * pad - yLocal.Z() * pad);

        const gp_Ax2 boxAx(boxOrigin, cutMain, refDir);
        const TopoDS_Shape skimBox = pr::box(boxAx,
                                             2.0 * pad,
                                             2.0 * pad,
                                             in.removal_mm + overhang);
        newShape    = pr::cut(wp.shape(), skimBox);
        geomChanged = true;
    }

    // Build signature.
    json params = {
        { "entry_face_id",     *entryId },
        { "removal_mm",        in.removal_mm },
        { "target_ra",         targetRa },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "target_ra",           targetRa },
        { "removal_mm",          in.removal_mm },
        { "geometry_changed",    geomChanged },
        { "entry_face_normal",   { outNorm.X(), outNorm.Y(), outNorm.Z() } },
        // We deliberately also include entry_area so a downstream consumer
        // can re-locate the polished face after re-import (geometry-based
        // recognition impossible — but a face matching the recorded area
        // and normal is the strongest candidate).
        { "entry_face_area_mm2", entryArea },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "polishing_wheel";
    tooling.tool_dia_mm       = 50.0;        // typical flap wheel
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "felt_diamond_paste";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 2000.0;      // very high surface speed
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = entryArea * in.removal_mm;
    tooling.est_cycle_time_s  = std::max(5.0, entryArea / 100.0);
    tooling.extra = {
        { "target_ra",     targetRa },
        { "finish_grade",  "mirror" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::polish_op applied: removal={} target_ra={} faces {}→{}",
                  in.removal_mm, targetRa,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Without metadata, polishing is invisible to B-rep (Ra is a sub-OCCT
// surface property).  We can ONLY recognize a polish_op by replaying
// signatures previously stamped onto the workpiece.

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

}  // namespace koocadcam::skill::polish_op
