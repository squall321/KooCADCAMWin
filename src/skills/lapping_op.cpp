// @lat: [[engine/skills#lapping_op]]

#include "lapping_op.hpp"

#include "Workpiece.hpp"
#include "face_milling.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::lapping_op {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.removal_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "lapping_op removal_mm must be > 0");
    }
    if (in.removal_mm > 0.0 && in.removal_mm < 0.0001) {
        r.add("DFM-LP-MIN", "error",
              "lapping_op removal_mm " + std::to_string(in.removal_mm) +
              " mm < min 0.0001 mm — below abrasive-compound cut threshold");
    }
    if (in.removal_mm > 0.005) {
        r.add("DFM-LP-MAX", "error",
              "lapping_op removal_mm " + std::to_string(in.removal_mm) +
              " mm > max 0.005 mm — exceeds single-pass lap limit "
              "(use surface_grind or honing_op for more removal)");
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
        std::string msg = "lapping_op DFM failed:";
        for (const auto& f : dfm.findings) {
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        }
        throw SkillError(msg);
    }

    // 2) Geometric path selection.
    //
    //    When removal_mm is below the coarse modelling threshold, the
    //    Boolean cut would either produce a degenerate shape or be
    //    indistinguishable from the input.  We skip the cut entirely and
    //    treat the operation as metadata-only — the signature still
    //    records the lapping intent for downstream CAM / process planning.
    const bool isGeometricNoOp = (in.removal_mm < kGeometricThresholdMm);

    std::shared_ptr<Workpiece> wpNew;
    json entryFaceMeta = json::object();
    double cutStockRemovedMm3 = 0.0;
    double cutCycleTimeS = 0.0;

    if (isGeometricNoOp) {
        // Geometric no-op: clone the workpiece, no Boolean.
        wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());

        // Try to record which face was the target, for downstream tooling.
        auto entryId = wp.resolve(in.entry_face);
        if (entryId && wp.isFacePlanar(*entryId)) {
            const gp_Dir n = wp.faceNormal(*entryId);
            entryFaceMeta = {
                { "entry_face_id",     *entryId },
                { "entry_face_normal", { n.X(), n.Y(), n.Z() } },
            };
            // Stock removed (theoretical): face area × removal_mm.
            GProp_GProps props;
            BRepGProp::SurfaceProperties(wp.face(*entryId), props);
            cutStockRemovedMm3 = props.Mass() * in.removal_mm;
        }
        cutCycleTimeS = 30.0;        // a lapping pass is slow regardless
    } else {
        // Removal is above threshold: delegate to face_milling for the cut.
        face_milling::Input fmIn;
        fmIn.entry_face = in.entry_face;
        fmIn.depth_mm   = in.removal_mm;
        SkillOutput fmOut = face_milling::apply(wp, fmIn);

        wpNew = std::make_shared<Workpiece>(fmOut.workpiece->shape(), wp.material());
        if (fmOut.signature.params.contains("entry_face_normal")) {
            entryFaceMeta["entry_face_normal"] = fmOut.signature.params["entry_face_normal"];
        }
        if (fmOut.signature.params.contains("entry_face_id")) {
            entryFaceMeta["entry_face_id"] = fmOut.signature.params["entry_face_id"];
        }
        cutStockRemovedMm3 = fmOut.signature.tooling.stock_removed_mm3;
        cutCycleTimeS      = fmOut.signature.tooling.est_cycle_time_s * 20.0;
    }

    // 3) Signature
    auto entryId = wp.resolve(in.entry_face);
    json params = {
        { "entry_face_id",     entryId ? *entryId : -1 },
        { "removal_mm",        in.removal_mm },
        { "lapping_compound",  in.lapping_compound },
        { "geometric_no_op",   isGeometricNoOp },
    };
    if (!entryFaceMeta.is_null() && !entryFaceMeta.empty()) {
        for (auto it = entryFaceMeta.begin(); it != entryFaceMeta.end(); ++it) {
            params[it.key()] = it.value();
        }
    }
    json pattern = {
        { "kind",                kSkillId },
        { "removal_mm",          in.removal_mm },
        { "lapping_compound",    in.lapping_compound },
        { "geometric_no_op",     isGeometricNoOp },
    };
    if (entryFaceMeta.contains("entry_face_normal")) {
        pattern["entry_face_normal"] = entryFaceMeta["entry_face_normal"];
    }

    ToolingMeta tooling;
    tooling.tool_type         = "lap_plate";
    tooling.tool_dia_mm       = 300.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "cast_iron";       // standard lap plate substrate
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 40.0;              // very slow, large area contact
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = cutStockRemovedMm3;
    tooling.est_cycle_time_s  = std::max(20.0, cutCycleTimeS);
    tooling.extra = json{
        { "lapping_compound",  in.lapping_compound },
        { "abrasive_size_um",  3.0 },
        { "carrier_fluid",     "kerosene" },
        { "operation_type",    "flat_lap" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    wpNew->addFeature(sig);

    spdlog::debug("skill::lapping_op applied: removal={} mm compound='{}' "
                  "geometric_no_op={} faces {}→{}",
                  in.removal_mm, in.lapping_compound,
                  isGeometricNoOp,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A lapped surface looks like any other planar face — geometry-based
// recognition is impossible.  recognize() returns empty.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    (void)wp;
    return {};
}

}  // namespace koocadcam::skill::lapping_op
