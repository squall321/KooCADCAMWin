// @lat: [[engine/skills#lens_polish]]

#include "lens_polish.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>

namespace koocadcam::skill::lens_polish {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

namespace {

bool isKnownCompound(const std::string& c)
{
    return c == "cerium_oxide"
        || c == "colloidal_silica"
        || c == "diamond_slurry"
        || c == "alumina";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────
//
// Roughness gates:
//   target_roughness_nm ≤ 0           → error  (DFM-INPUT)
//   target_roughness_nm < 0.2 nm      → error  (DFM-LENS-POLISH-ROUGH —
//                                                below practical pitch-lap floor)
//   target_roughness_nm > 50 nm       → info   (DFM-LENS-POLISH-ROUGH —
//                                                so coarse it suggests grind, not polish)
//   target_roughness_nm in (0.2, 50]  → pass

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_roughness_nm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lens_polish target_roughness_nm must be > 0 (got " +
              std::to_string(in.target_roughness_nm) + ")");
    } else {
        if (in.target_roughness_nm < 0.2) {
            r.add("DFM-LENS-POLISH-ROUGH", "error",
                  "lens_polish target_roughness_nm " +
                  std::to_string(in.target_roughness_nm) +
                  " < 0.2 nm — below practical pitch-lap polish floor");
        } else if (in.target_roughness_nm > 50.0) {
            r.add("DFM-LENS-POLISH-ROUGH", "info",
                  "lens_polish target_roughness_nm " +
                  std::to_string(in.target_roughness_nm) +
                  " > 50 nm — typical of a grind, not a polish; consider lens_grind");
        }
    }

    if (!isKnownCompound(in.polishing_compound)) {
        r.add("DFM-LENS-POLISH-COMPOUND", "info",
              "lens_polish polishing_compound '" + in.polishing_compound +
              "' not in known set "
              "(cerium_oxide | colloidal_silica | diamond_slurry | alumina) "
              "— process plan will treat as custom slurry");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lens_polish DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("lens_polish: entry_face datum unresolved");

    const std::string compound = in.polishing_compound.empty()
                               ? "cerium_oxide"
                               : in.polishing_compound;

    const std::string polish_class =
        in.target_roughness_nm <= 1.0 ? "super_polish"
      : in.target_roughness_nm <= 5.0 ? "precision_polish"
                                      : "commercial_polish";

    json params = {
        { "entry_face_id",       *entryId },
        { "target_roughness_nm", in.target_roughness_nm },
        { "polishing_compound",  compound },
    };
    json pattern = {
        { "kind",                "lens_polish" },
        { "target_face_id",      *entryId },
        { "target_roughness_nm", in.target_roughness_nm },
        { "polishing_compound",  compound },
        { "polish_class",        polish_class },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "pitch_lap";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = compound;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Polish time scales inversely with target roughness; floor 5 min.
    tooling.est_cycle_time_s  = std::max(5.0 * 60.0,
                                         300.0 / in.target_roughness_nm * 60.0);
    tooling.extra = {
        { "process",             "lens_polish" },
        { "target_roughness_nm", in.target_roughness_nm },
        { "polishing_compound",  compound },
        { "polish_class",        polish_class },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug("skill::lens_polish applied: face={} roughness={}nm compound={}",
                  *entryId, in.target_roughness_nm, compound);

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::lens_polish
