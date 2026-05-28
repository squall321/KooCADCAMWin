// @lat: [[engine/skills#electroplate]]

#include "electroplate.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::electroplate {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

static const char* fitToString(cc::PlatingFit f)
{
    switch (f) {
    case cc::PlatingFit::Good:        return "good";
    case cc::PlatingFit::NeedsStrike: return "needs_strike";
    case cc::PlatingFit::Unknown:     return "unknown";
    }
    return "unknown";
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.coating_thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "electroplate coating_thickness_um must be > 0");
    } else {
        if (in.coating_thickness_um < 1.0) {
            r.add("DFM-EP-THK", "error",
                  "electroplate coating_thickness_um " +
                  std::to_string(in.coating_thickness_um) +
                  " < 1 μm — below practical electroplate lower bound");
        }
        if (in.coating_thickness_um > 200.0) {
            r.add("DFM-EP-THK", "error",
                  "electroplate coating_thickness_um " +
                  std::to_string(in.coating_thickness_um) +
                  " > 200 μm — above typical electroplate ceiling "
                  "(use electroforming for thicker deposits)");
        }
    }

    if (!cc::isKnownPlatingMetal(in.plating_metal)) {
        r.add("DFM-EP-METAL", "info",
              "electroplate plating_metal '" + in.plating_metal +
              "' not in {nickel, chrome, zinc, copper, gold} — process plan "
              "will treat as custom plating chemistry");
    } else {
        const auto* pc = cc::findPlating(in.plating_metal);
        const std::string base = in.base_material.empty()
                                 ? wp.material()
                                 : in.base_material;
        const cc::PlatingFit fit = cc::classifyPlatingFit(*pc, base);
        if (fit == cc::PlatingFit::NeedsStrike) {
            r.add("DFM-EP-FIT", "info",
                  "electroplate " + in.plating_metal + " on '" + base +
                  "' requires intermediate strike-plate "
                  "(typical: zincate or thin Ni underlayer)");
        } else if (fit == cc::PlatingFit::Unknown) {
            r.add("DFM-EP-FIT", "info",
                  "electroplate " + in.plating_metal + " on '" + base +
                  "' compatibility not in table — verify bath chemistry");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "electroplate DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("electroplate: entry_face datum unresolved");

    const std::string base = in.base_material.empty()
                             ? wp.material()
                             : in.base_material;
    const cc::PlatingCompatibility* pc = cc::findPlating(in.plating_metal);
    const cc::PlatingFit fit = pc ? cc::classifyPlatingFit(*pc, base)
                                  : cc::PlatingFit::Unknown;
    const double hardnessHv = pc ? pc->hardness_hv : 0.0;

    json params = {
        { "entry_face_id",        *entryId },
        { "plating_metal",        in.plating_metal },
        { "coating_thickness_um", in.coating_thickness_um },
        { "base_material",        base },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "plating_metal",        in.plating_metal },
        { "coating_thickness_um", in.coating_thickness_um },
        { "base_material",        base },
        { "plating_fit",          fitToString(fit) },
        { "deposit_hardness_hv",  hardnessHv },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "electroplate_bath";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.plating_metal + "_anode";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Plating rate ~ 0.5 μm/min typical bath current density;
    // gold/chrome are slower — clamp by metal type.
    double rate_um_per_min = 0.5;
    if (in.plating_metal == "chrome") rate_um_per_min = 0.2;
    if (in.plating_metal == "gold")   rate_um_per_min = 0.1;
    tooling.est_cycle_time_s  = std::max(30.0,
                                         in.coating_thickness_um
                                       / rate_um_per_min * 60.0);
    tooling.extra = {
        { "process",              "electroplate" },
        { "plating_metal",        in.plating_metal },
        { "coating_thickness_um", in.coating_thickness_um },
        { "base_material",        base },
        { "plating_fit",          fitToString(fit) },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug(
        "skill::electroplate applied: face={} metal={} thk={}μm fit={}",
        *entryId, in.plating_metal, in.coating_thickness_um, fitToString(fit));

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::electroplate
