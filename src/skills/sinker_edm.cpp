// @lat: [[engine/skills#sinker_edm]]

#include "sinker_edm.hpp"

#include "Workpiece.hpp"
#include "mill_circular_pocket.hpp"
#include "mill_rect_pocket.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::sinker_edm {

using nlohmann::json;

namespace {

constexpr const char* kShapeRect = "rect_pocket";
constexpr const char* kShapeCyl  = "cylindrical_cavity";

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.electrode_shape != kShapeRect && in.electrode_shape != kShapeCyl) {
        r.add("DFM-INPUT", "error",
              "sinker_edm electrode_shape '" + in.electrode_shape +
              "' not recognised — use 'rect_pocket' or 'cylindrical_cavity'");
        return r;  // can't validate shape-specific dims without a shape
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "sinker_edm depth_mm must be > 0");
    }
    if (in.depth_mm > 50.0) {
        r.add("DFM-EDM-DEPTH", "warning",
              "sinker_edm depth_mm " + std::to_string(in.depth_mm) +
              " > 50 mm — exceeds typical sinker stroke; secondary electrode "
              "/ deeper flushing required");
    }

    if (in.electrode_shape == kShapeRect) {
        if (!in.dims.contains("length_mm") || !in.dims.contains("width_mm")) {
            r.add("DFM-INPUT", "error",
                  "sinker_edm rect_pocket needs dims { length_mm, width_mm }");
        } else {
            const double L = in.dims.value("length_mm", 0.0);
            const double W = in.dims.value("width_mm", 0.0);
            if (L < 0.5)
                r.add("DFM-EDM-DIM", "error",
                      "sinker_edm length_mm " + std::to_string(L) + " < 0.5 mm");
            if (W < 0.5)
                r.add("DFM-EDM-DIM", "error",
                      "sinker_edm width_mm " + std::to_string(W) + " < 0.5 mm");
        }
    } else if (in.electrode_shape == kShapeCyl) {
        if (!in.dims.contains("dia_mm")) {
            r.add("DFM-INPUT", "error",
                  "sinker_edm cylindrical_cavity needs dims { dia_mm }");
        } else {
            const double D = in.dims.value("dia_mm", 0.0);
            if (D < 0.5)
                r.add("DFM-EDM-DIM", "error",
                      "sinker_edm dia_mm " + std::to_string(D) + " < 0.5 mm");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sinker_edm DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    SkillOutput inner;

    if (in.electrode_shape == kShapeRect) {
        mill_rect_pocket::Input rp;
        rp.entry_face   = in.entry_face;
        rp.center_x_mm  = in.position_x_mm;
        rp.center_y_mm  = in.position_y_mm;
        rp.axis_dir     = gp_Dir(0.0, 0.0, -1.0);
        rp.length_mm    = in.dims.value("length_mm", 0.0);
        rp.width_mm     = in.dims.value("width_mm",  0.0);
        rp.depth_mm     = in.depth_mm;
        rp.corner_r_mm  = 0.0;   // EDM does NOT leave tool-radius fillets
        inner = mill_rect_pocket::apply(wp, rp);
    } else if (in.electrode_shape == kShapeCyl) {
        mill_circular_pocket::Input cp;
        cp.entry_face          = in.entry_face;
        cp.position_x_mm       = in.position_x_mm;
        cp.position_y_mm       = in.position_y_mm;
        cp.axis_dir            = gp_Dir(0.0, 0.0, -1.0);
        cp.diameter_mm         = in.dims.value("dia_mm", 0.0);
        cp.depth_mm            = in.depth_mm;
        cp.bottom_corner_r_mm  = 0.0;
        inner = mill_circular_pocket::apply(wp, cp);
    } else {
        throw SkillError("sinker_edm: unreachable electrode_shape");
    }

    // Stamp the signature with sinker_edm metadata over the underlying one.
    FeatureSignature sig = inner.signature;
    sig.skill_id = kSkillId;
    sig.pattern["kind"]              = kSkillId;
    sig.pattern["electrode_shape"]   = in.electrode_shape;
    sig.pattern["underlying_skill"]  = inner.signature.skill_id;

    sig.params = json{
        { "entry_face_id",   inner.signature.params.value("entry_face_id", -1) },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "electrode_shape", in.electrode_shape },
        { "dims",            in.dims },
        { "depth_mm",        in.depth_mm },
    };

    sig.tooling.tool_type        = "edm_electrode";
    sig.tooling.tool_material    = "copper";          // or "graphite"
    sig.tooling.flute_count      = 0;                 // no flutes; spark erosion
    sig.tooling.cutting_speed_sfm = 0.0;              // not applicable
    sig.tooling.feed_per_tooth_mm = 0.0;
    // EDM removes material slowly; rough cycle time = stock_removed / 5 mm³/s
    sig.tooling.est_cycle_time_s = std::max(10.0,
        sig.tooling.stock_removed_mm3 / 5.0);
    sig.tooling.extra["edm_kind"] = "sinker";
    sig.tooling.extra["machining_constraint"] =
        "EDM die-sink — sharp internal corners possible; long cycle time";

    // Replace the underlying feature in the workpiece with the EDM-stamped one.
    auto wpOut = inner.workpiece;
    // Remove the inner skill's feature and add the sinker_edm version.
    // (Workpiece doesn't expose a remove API, so we rebuild the feature list.)
    std::vector<FeatureSignature> features;
    for (const auto& prev : wpOut->features()) {
        if (prev.skill_id == inner.signature.skill_id) continue;
        features.push_back(prev);
    }
    features.push_back(sig);
    wpOut->setFeatures(features);

    spdlog::debug("skill::sinker_edm applied: shape={} depth={} faces {}→{}",
                  in.electrode_shape, in.depth_mm,
                  wp.faceCount(), wpOut->faceCount());

    return SkillOutput{ wpOut, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Sinker EDM is geometrically indistinguishable from a milled pocket
// without metadata — the only difference is sharp corners (corner_r = 0).
// We return ZERO candidates here.  The orchestrator may infer sinker_edm
// from the underlying pocket recognizer's results + metadata replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    // Metadata replay only — if a prior sinker_edm signature is in
    // wp.features(), expose it as a high-confidence candidate.
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

}  // namespace koocadcam::skill::sinker_edm
