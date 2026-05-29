// @lat: [[engine/skills#vsat_assemble]]

#include "vsat_assemble.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::vsat_assemble {

using nlohmann::json;

namespace {

bool isKnownPolarization(const std::string& p)
{
    return p == "linear_h" || p == "linear_v" || p == "rhcp" || p == "lhcp";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.dish_dia_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": dish_dia_m must be > 0 (got " +
              std::to_string(in.dish_dia_m) + ")");
    } else if (in.dish_dia_m < 0.6 || in.dish_dia_m > 3.7) {
        r.add("DFM-VSAT-DIA", "warning",
              std::string(kSkillId) + ": dish_dia_m " +
              std::to_string(in.dish_dia_m) +
              " outside typical VSAT range [0.6, 3.7] m");
    }

    if (in.transponder.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": transponder must be non-empty");
    }

    if (!isKnownPolarization(in.polarization)) {
        r.add("DFM-VSAT-POL", "info",
              std::string(kSkillId) + ": polarization '" + in.polarization +
              "' not in {linear_h, linear_v, rhcp, lhcp}");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vsat_assemble DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "dish_dia_m",    in.dish_dia_m },
        { "transponder",   in.transponder },
        { "polarization",  in.polarization },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "dish_dia_m",       in.dish_dia_m },
        { "transponder",      in.transponder },
        { "polarization",     in.polarization },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "vsat_assembly_kit";
    tooling.tool_dia_mm       = in.dish_dia_m * 1000.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Larger dishes take longer: base 30 min + 20 min per metre of dia.
    tooling.est_cycle_time_s  = 1800.0 + 1200.0 * in.dish_dia_m;
    tooling.extra = {
        { "process",          "vsat_terminal_assemble" },
        { "dish_dia_m",       in.dish_dia_m },
        { "transponder",      in.transponder },
        { "polarization",     in.polarization },
        { "process_category", "telecom_install" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::vsat_assemble applied: dia={}m xpnder={} pol={}",
        in.dish_dia_m, in.transponder, in.polarization);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric trace on the workpiece.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != std::string(kSkillId)) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::vsat_assemble
