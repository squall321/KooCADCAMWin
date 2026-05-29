// @lat: [[engine/skills#electroform_jewelry]]

#include "electroform_jewelry.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::electroform_jewelry {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownMetals()
{
    static const std::unordered_set<std::string> s = {
        "gold", "silver", "copper", "nickel",
    };
    return s;
}

// Typical industrial bath deposit rates (µm / hour).
double depositRateFor(const std::string& metal)
{
    if (metal == "gold")   return 18.0;
    if (metal == "silver") return 25.0;
    if (metal == "copper") return 40.0;
    if (metal == "nickel") return 22.0;
    return 20.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.deposit_metal.empty()) {
        r.add("DFM-INPUT", "error",
              "electroform_jewelry deposit_metal must be non-empty");
    } else if (knownMetals().count(in.deposit_metal) == 0) {
        r.add("DFM-DEPOSIT-METAL", "error",
              "electroform_jewelry deposit_metal '" + in.deposit_metal +
              "' not in {gold, silver, copper, nickel}");
    }

    if (in.mandrel_id.empty()) {
        r.add("DFM-INPUT", "error",
              "electroform_jewelry mandrel_id must be non-empty");
    }

    if (in.thickness_um <= 0.0) {
        r.add("DFM-THICKNESS", "error",
              "electroform_jewelry thickness_um must be > 0 (got " +
              std::to_string(in.thickness_um) + ")");
    } else {
        if (in.thickness_um < 25.0) {
            r.add("DFM-THICKNESS-LO", "info",
                  "electroform_jewelry thickness_um " +
                  std::to_string(in.thickness_um) +
                  " < 25 — too thin, shape collapse risk after mandrel removal");
        }
        if (in.thickness_um > 1000.0) {
            r.add("DFM-THICKNESS-HI", "info",
                  "electroform_jewelry thickness_um " +
                  std::to_string(in.thickness_um) +
                  " > 1000 — switch to casting, deposition time prohibitive");
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
        std::string msg = "electroform_jewelry DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double rate           = depositRateFor(in.deposit_metal);
    const double expected_hours = (rate > 0.0) ? (in.thickness_um / rate) : 0.0;

    json params = {
        { "deposit_metal", in.deposit_metal },
        { "thickness_um",  in.thickness_um },
        { "mandrel_id",    in.mandrel_id },
    };

    json pattern = {
        { "kind",              kSkillId },
        { "deposit_metal",     in.deposit_metal },
        { "thickness_um",      in.thickness_um },
        { "mandrel_id",        in.mandrel_id },
        { "deposit_rate_um_h", rate },
        { "expected_hours",    expected_hours },
        { "process_family",    "electroforming_jewelry" },
        { "geometry_changed",  false },
        { "is_process_only",   true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "electroforming_bath";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    if      (in.deposit_metal == "gold")   tooling.tool_material = "Au_cyanide_bath";
    else if (in.deposit_metal == "silver") tooling.tool_material = "Ag_cyanide_bath";
    else if (in.deposit_metal == "copper") tooling.tool_material = "Cu_sulfate_bath";
    else if (in.deposit_metal == "nickel") tooling.tool_material = "Ni_watts_bath";
    else                                    tooling.tool_material = "electrolyte_bath";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = expected_hours * 3600.0;
    tooling.extra = {
        { "process",           "electroforming" },
        { "deposit_metal",     in.deposit_metal },
        { "thickness_um",      in.thickness_um },
        { "mandrel_id",        in.mandrel_id },
        { "deposit_rate_um_h", rate },
        { "expected_hours",    expected_hours },
        { "process_category",  "jewelry" },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::electroform_jewelry applied: metal={} t={}um mandrel={}",
                  in.deposit_metal, in.thickness_um, in.mandrel_id);

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
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::electroform_jewelry
