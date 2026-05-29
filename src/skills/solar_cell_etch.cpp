// @lat: [[engine/skills#solar_cell_etch]]

#include "solar_cell_etch.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::solar_cell_etch {

using nlohmann::json;

namespace {

bool isKnownEtchant(const std::string& e)
{
    return e == "KOH" || e == "NaOH" || e == "HF_HNO3" || e == "TMAH";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.cell_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cell_size_mm must be > 0 (got " +
              std::to_string(in.cell_size_mm) + ")");
    } else if (in.cell_size_mm < 125.0 || in.cell_size_mm > 230.0) {
        r.add("DFM-PV-CELL-SIZE", "info",
              std::string(kSkillId) + ": cell_size_mm " +
              std::to_string(in.cell_size_mm) +
              " outside typical M2..M12 wafer range [125, 230] mm");
    }

    if (in.etchant.empty()) {
        r.add("DFM-INPUT", "warning",
              std::string(kSkillId) + ": etchant empty — defaulting to KOH");
    } else if (!isKnownEtchant(in.etchant)) {
        r.add("DFM-PV-ETCHANT", "info",
              std::string(kSkillId) + ": etchant '" + in.etchant +
              "' not in {KOH, NaOH, HF_HNO3, TMAH}");
    }

    if (in.time_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": time_min must be > 0 (got " +
              std::to_string(in.time_min) + ")");
    } else if (in.time_min > 60.0) {
        r.add("DFM-PV-ETCH-TIME", "info",
              std::string(kSkillId) + ": time_min " +
              std::to_string(in.time_min) +
              " > 60 min — over-etch risk, verify recipe");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "solar_cell_etch DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string etchant = in.etchant.empty() ? "KOH" : in.etchant;

    json params = {
        { "cell_size_mm", in.cell_size_mm },
        { "etchant",      etchant },
        { "time_min",     in.time_min },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "cell_size_mm",     in.cell_size_mm },
        { "etchant",          etchant },
        { "time_min",         in.time_min },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "etch_bath";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "PTFE_tank";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.time_min * 60.0;
    tooling.extra = {
        { "process",          "wet_chemical_etch" },
        { "etchant",          etchant },
        { "cell_size_mm",     in.cell_size_mm },
        { "process_category", "pv_cell_fabrication" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::solar_cell_etch applied: cell={}mm etchant={} t={}min",
        in.cell_size_mm, etchant, in.time_min);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Sub-micron surface texture is below B-rep resolution — metadata-only.

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

}  // namespace koocadcam::skill::solar_cell_etch
