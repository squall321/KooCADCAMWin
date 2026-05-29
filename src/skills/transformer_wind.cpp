// @lat: [[engine/skills#transformer_wind]]

#include "transformer_wind.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::transformer_wind {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.primary_turns <= 0) {
        r.add("DFM-INPUT", "error",
              "transformer_wind primary_turns must be > 0 (got " +
              std::to_string(in.primary_turns) + ")");
    }
    if (in.secondary_turns <= 0) {
        r.add("DFM-INPUT", "error",
              "transformer_wind secondary_turns must be > 0 (got " +
              std::to_string(in.secondary_turns) + ")");
    }
    if (in.wire_dia_mm <= 0.05 || in.wire_dia_mm > 10.0) {
        r.add("DFM-XFMR-WIRE", "error",
              "transformer_wind wire_dia_mm " +
              std::to_string(in.wire_dia_mm) +
              " outside winding-wire range (0.05, 10.0] mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "transformer_wind DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double ratio =
        static_cast<double>(in.primary_turns) /
        static_cast<double>(in.secondary_turns);

    json params = {
        { "primary_turns",   in.primary_turns },
        { "secondary_turns", in.secondary_turns },
        { "wire_dia_mm",     in.wire_dia_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "primary_turns",    in.primary_turns },
        { "secondary_turns",  in.secondary_turns },
        { "wire_dia_mm",      in.wire_dia_mm },
        { "turns_ratio",      ratio },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "transformer_winder";
    tooling.tool_dia_mm       = in.wire_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "enameled_copper";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 0.4 s per turn (two windings).
    tooling.est_cycle_time_s  = 0.4 *
        static_cast<double>(in.primary_turns + in.secondary_turns);
    tooling.extra = {
        { "process",         "transformer_winding" },
        { "primary_turns",   in.primary_turns },
        { "secondary_turns", in.secondary_turns },
        { "wire_dia_mm",     in.wire_dia_mm },
        { "turns_ratio",     ratio },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::transformer_wind applied: N1={} N2={} d={} ratio={}",
                  in.primary_turns, in.secondary_turns, in.wire_dia_mm, ratio);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata-only) ──────────────────────────────────────────

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

}  // namespace koocadcam::skill::transformer_wind
