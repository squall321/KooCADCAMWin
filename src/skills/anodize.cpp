// @lat: [[engine/skills#anodize]]

#include "anodize.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::anodize {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.coating_thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "anodize coating_thickness_um must be > 0");
    } else {
        if (in.coating_thickness_um < 5.0) {
            r.add("DFM-ANODIZE-THK", "error",
                  "anodize coating_thickness_um " +
                  std::to_string(in.coating_thickness_um) +
                  " < 5 μm — below practical Type I lower bound");
        }
        if (in.coating_thickness_um > 75.0) {
            r.add("DFM-ANODIZE-THK", "error",
                  "anodize coating_thickness_um " +
                  std::to_string(in.coating_thickness_um) +
                  " > 75 μm — exceeds typical Type III hard-anodize ceiling");
        }
    }

    if (in.color.empty()) {
        r.add("DFM-INPUT", "warning",
              "anodize color empty — defaulting to natural");
    } else if (!cc::isKnownAnodizeColor(in.color)) {
        r.add("DFM-ANODIZE-COLOR", "info",
              "anodize color '" + in.color +
              "' not in known palette — process plan will treat as custom dye");
    }

    if (!cc::isKnownAnodizeSeal(in.seal_type)) {
        r.add("DFM-ANODIZE-SEAL", "info",
              "anodize seal_type '" + in.seal_type +
              "' unrecognised — expected hot_water | nickel_acetate | none");
    }

    if (!cc::isAluminumFamily(wp.material())) {
        r.add("DFM-ANODIZE-MATERIAL", "info",
              "anodize substrate material '" + wp.material() +
              "' not aluminum-family — anodize bath chemistry may not bond "
              "(consider electroplate or pvd_coat instead)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "anodize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("anodize: entry_face datum unresolved");

    const std::string color = in.color.empty() ? "natural" : in.color;
    const cc::RgbHint rgb   = cc::anodizeRgbHint(color);

    json params = {
        { "entry_face_id",        *entryId },
        { "coating_thickness_um", in.coating_thickness_um },
        { "color",                color },
        { "seal_type",            in.seal_type },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "coating_thickness_um", in.coating_thickness_um },
        { "color",                color },
        { "color_rgb_hint",       { rgb.r, rgb.g, rgb.b } },
        { "seal_type",            in.seal_type },
        { "anodize_class",
          in.coating_thickness_um >= 50.0 ? "Type_III_hard"
                                          : "Type_II_standard" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "anodize_bath";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "sulfuric_acid_electrolyte";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // additive (no removal)
    // Anodize growth ~ 1 μm per minute at standard current density —
    // total dunk time ≈ thickness_um × ~1 min/μm, clamped 5 min min.
    tooling.est_cycle_time_s  = std::max(5.0 * 60.0,
                                         in.coating_thickness_um * 60.0);
    tooling.extra = {
        { "process",              "Type_II_anodize" },
        { "coating_thickness_um", in.coating_thickness_um },
        { "color",                color },
        { "seal_type",            in.seal_type },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug("skill::anodize applied: face={} thk={}μm color={} seal={}",
                  *entryId, in.coating_thickness_um, color, in.seal_type);

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::anodize
