// @lat: [[engine/skills#surface_grind]]

#include "surface_grind.hpp"

#include "Workpiece.hpp"
#include "face_milling.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::surface_grind {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.removal_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "surface_grind removal_mm must be > 0");
    }
    if (in.removal_mm > 0.0 && in.removal_mm < 0.005) {
        r.add("DFM-SG-MIN", "error",
              "surface_grind removal_mm " + std::to_string(in.removal_mm) +
              " mm < min 0.005 mm — below grinding-wheel cut threshold");
    }
    if (in.removal_mm > 0.2) {
        r.add("DFM-SG-MAX", "error",
              "surface_grind removal_mm " + std::to_string(in.removal_mm) +
              " mm > max 0.2 mm — exceeds single-pass surface-grind limit "
              "(use face_milling for larger removal)");
    }

    // removal_mm ≤ workpiece_thickness × 0.1 (workpiece thickness = bbox min dim)
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
    if (thickness > 0.0 && in.removal_mm > thickness * 0.1) {
        r.add("DFM-SG-THK", "warning",
              "surface_grind removal_mm " + std::to_string(in.removal_mm) +
              " mm > 10% of workpiece thickness " + std::to_string(thickness) +
              " mm — risk of distortion / heat checking");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "surface_grind DFM failed:";
        for (const auto& f : dfm.findings) {
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        }
        throw SkillError(msg);
    }

    // 2) Delegate to face_milling for the geometric removal.  Same input
    //    semantics: planar face + downward depth.
    face_milling::Input fmIn;
    fmIn.entry_face = in.entry_face;
    fmIn.depth_mm   = in.removal_mm;
    SkillOutput fmOut = face_milling::apply(wp, fmIn);

    // 3) Restamp the signature with surface_grind-specific metadata.
    //    The shape is geometrically identical to a fine face mill pass; the
    //    process distinction lives in the signature + tooling fields.
    auto entryId = wp.resolve(in.entry_face);
    json params = {
        { "entry_face_id",   entryId ? *entryId : -1 },
        { "removal_mm",      in.removal_mm },
        { "surface_finish",  in.surface_finish },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "removal_mm",            in.removal_mm },
        { "surface_finish",        in.surface_finish },
        { "parallel_face_present", true },
    };
    // Copy the face_milling-resolved normal forward, if available.
    if (fmOut.signature.params.contains("entry_face_normal")) {
        pattern["entry_face_normal"] = fmOut.signature.params["entry_face_normal"];
        params["entry_face_normal"]  = fmOut.signature.params["entry_face_normal"];
    }

    ToolingMeta tooling;
    tooling.tool_type         = "grinding_wheel";
    tooling.tool_dia_mm       = 200.0;          // typical 8" surface grinder wheel
    tooling.tool_length_mm    = 25.0;           // wheel width
    tooling.tool_material     = "aluminum_oxide";
    tooling.flute_count       = 0;              // abrasive, not toothed
    tooling.cutting_speed_sfm = 5500.0;         // surface grinder wheels run fast
    tooling.feed_per_tooth_mm = 0.0;            // table-feed, not per-tooth
    // Stock removed ≈ face area × removal_mm — face_milling already
    // computed face_area × depth, with depth == removal_mm.
    tooling.stock_removed_mm3 = fmOut.signature.tooling.stock_removed_mm3;
    // Grinding feed rate ~ 10× slower than face milling at the same MRR.
    tooling.est_cycle_time_s  = fmOut.signature.tooling.est_cycle_time_s * 8.0;
    tooling.extra = json{
        { "surface_finish",  in.surface_finish },
        { "abrasive_grit",   "60-grit" },
        { "coolant",         "flood" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // 4) Replace the face_milling feature on the output workpiece with ours.
    //    fmOut.workpiece currently has the face_milling signature appended;
    //    we wrap by constructing a fresh workpiece around the same shape so
    //    that only the surface_grind history is recorded.
    auto wpNew = std::make_shared<Workpiece>(fmOut.workpiece->shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::surface_grind applied: removal={} mm finish='{}' "
                  "faces {}→{}",
                  in.removal_mm, in.surface_finish,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A surface-ground face has the same topology as the original planar face —
// at our coarse modelling fidelity (typical OCCT tolerance ≈ 1e-3 mm and our
// removal range is 5e-3 … 2e-1 mm) there is no geometric marker that
// distinguishes "ground" from "milled finish."  Without process metadata
// surface_grind cannot be detected.  Return empty.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    (void)wp;
    return {};   // metadata-only differentiation; no shape-based recognition
}

}  // namespace koocadcam::skill::surface_grind
