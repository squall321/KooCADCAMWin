// @lat: [[engine/skills#thin_film_deposit]]

#include "thin_film_deposit.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::thin_film_deposit {

using nlohmann::json;

namespace {

bool isKnownProcess(const std::string& s)
{
    return s == "CVD" || s == "PVD" || s == "ALD";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isKnownProcess(in.process_type)) {
        r.add("DFM-DEPO-TYPE", "error",
              std::string(kSkillId) + ": process_type '" + in.process_type +
              "' must be CVD | PVD | ALD");
    }

    if (in.thickness_nm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": thickness_nm must be > 0 (got " +
              std::to_string(in.thickness_nm) + ")");
    }

    if (in.deposit_rate_nm_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": deposit_rate_nm_min must be > 0 (got " +
              std::to_string(in.deposit_rate_nm_min) + ")");
    }

    if (in.process_type == "ALD" && in.thickness_nm > 100.0) {
        r.add("DFM-DEPO-ALD-THICK", "info",
              std::string(kSkillId) + ": ALD with thickness_nm " +
              std::to_string(in.thickness_nm) +
              " > 100 nm — throughput will be very slow (consider CVD/PVD)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "thin_film_deposit DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double cycle_min =
        (in.deposit_rate_nm_min > 0.0)
            ? (in.thickness_nm / in.deposit_rate_nm_min) : 0.0;

    json params = {
        { "process_type",        in.process_type },
        { "thickness_nm",        in.thickness_nm },
        { "deposit_rate_nm_min", in.deposit_rate_nm_min },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "process_type",        in.process_type },
        { "thickness_nm",        in.thickness_nm },
        { "deposit_rate_nm_min", in.deposit_rate_nm_min },
        { "cycle_time_min",      cycle_min },
        { "is_additive",         true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "deposition_chamber";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "process_gas_precursor";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = cycle_min * 60.0;
    tooling.extra = {
        { "process",         in.process_type + "_deposit" },
        { "thickness_nm",    in.thickness_nm },
        { "is_additive",     true },
        { "dfm_findings",    findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::thin_film_deposit applied: {} thk={}nm rate={}nm/min",
                  in.process_type, in.thickness_nm, in.deposit_rate_nm_min);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

}  // namespace koocadcam::skill::thin_film_deposit
