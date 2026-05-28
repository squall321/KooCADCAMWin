// @lat: [[engine/skills#etch_silicon]]

#include "etch_silicon.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::etch_silicon {

using nlohmann::json;

namespace {

bool isKnownEtchType(const std::string& s)
{
    return s == "DRIE" || s == "wet" || s == "dry";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // ENFORCED: depth must be strictly positive.
    if (in.depth_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": depth_um must be > 0 (got " +
              std::to_string(in.depth_um) + ")");
    }

    if (in.selectivity <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": selectivity must be > 0 (got " +
              std::to_string(in.selectivity) + ")");
    }

    if (in.mask_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": mask_id must be non-empty");
    }

    if (!isKnownEtchType(in.etch_type)) {
        r.add("DFM-ETCH-TYPE", "error",
              std::string(kSkillId) + ": etch_type '" + in.etch_type +
              "' must be DRIE | wet | dry");
    }

    // Selectivity check — info-level guidance: mask consumption risk.
    // If user etches deep with poor selectivity, mask will be eroded.
    if (in.depth_um > 0.0 && in.selectivity > 0.0 &&
        in.selectivity < 10.0 && in.depth_um > 50.0)
    {
        r.add("DFM-ETCH-SELECTIVITY", "info",
              std::string(kSkillId) + ": selectivity " +
              std::to_string(in.selectivity) +
              " with depth_um " + std::to_string(in.depth_um) +
              " > 50 μm — mask will likely be consumed before etch completes");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "etch_silicon DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Mask budget = selectivity · depth — required mask thickness to survive.
    const double mask_consumed_um = in.depth_um / in.selectivity;

    json params = {
        { "etch_type",   in.etch_type },
        { "depth_um",    in.depth_um },
        { "selectivity", in.selectivity },
        { "mask_id",     in.mask_id },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "etch_type",        in.etch_type },
        { "depth_um",         in.depth_um },
        { "selectivity",      in.selectivity },
        { "mask_id",          in.mask_id },
        { "mask_consumed_um", mask_consumed_um },
        { "is_subtractive",   true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "etch_chamber";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (in.etch_type == "wet") ? "KOH_or_TMAH"
                              : (in.etch_type == "DRIE") ? "SF6_C4F8_plasma"
                              : "RIE_plasma";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // metadata-only; volume not modelled
    // Typical rates: DRIE 5 μm/min, wet 1 μm/min, dry 0.5 μm/min.
    const double rate_um_min = (in.etch_type == "DRIE") ? 5.0
                              : (in.etch_type == "wet")  ? 1.0 : 0.5;
    tooling.est_cycle_time_s  = (in.depth_um / rate_um_min) * 60.0;
    tooling.extra = {
        { "process",          in.etch_type + "_silicon_etch" },
        { "depth_um",         in.depth_um },
        { "mask_consumed_um", mask_consumed_um },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::etch_silicon applied: {} depth={}μm sel={} mask={}",
                  in.etch_type, in.depth_um, in.selectivity, in.mask_id);

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

}  // namespace koocadcam::skill::etch_silicon
