// @lat: [[engine/skills#solar_panel_assemble]]

#include "solar_panel_assemble.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <set>
#include <string>

namespace koocadcam::skill::solar_panel_assemble {

using nlohmann::json;

namespace {

const std::set<std::string>& knownEncapsulants()
{
    static const std::set<std::string> e{ "EVA", "POE", "TPO" };
    return e;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cell_count <= 0) {
        r.add("DFM-INPUT", "error",
              "solar_panel_assemble cell_count must be > 0 (got " +
              std::to_string(in.cell_count) + ")");
    }
    if (in.cell_size_mm < 125.0 || in.cell_size_mm > 230.0) {
        r.add("DFM-PV-CELL-SIZE", "error",
              "solar_panel_assemble cell_size_mm " +
              std::to_string(in.cell_size_mm) +
              " outside production range [125, 230] mm");
    }
    if (knownEncapsulants().find(in.encapsulant) == knownEncapsulants().end()) {
        r.add("DFM-PV-ENCAP", "info",
              "solar_panel_assemble encapsulant '" + in.encapsulant +
              "' not in {EVA, POE, TPO}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "solar_panel_assemble DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Single-cell active area (mm²).
    const double cell_area_mm2 = in.cell_size_mm * in.cell_size_mm;
    const double total_active_area_mm2 =
        cell_area_mm2 * static_cast<double>(in.cell_count);

    json params = {
        { "cell_count",   in.cell_count },
        { "cell_size_mm", in.cell_size_mm },
        { "encapsulant",  in.encapsulant },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "cell_count",              in.cell_count },
        { "cell_size_mm",            in.cell_size_mm },
        { "encapsulant",             in.encapsulant },
        { "cell_area_mm2",           cell_area_mm2 },
        { "total_active_area_mm2",   total_active_area_mm2 },
        { "geometry_changed",        false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "stringer_laminator";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 6 s tabbing+stringing per cell.
    tooling.est_cycle_time_s  = 6.0 * static_cast<double>(in.cell_count);
    tooling.extra = {
        { "process",                "pv_module_assembly" },
        { "cell_count",             in.cell_count },
        { "cell_size_mm",           in.cell_size_mm },
        { "encapsulant",            in.encapsulant },
        { "total_active_area_mm2",  total_active_area_mm2 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::solar_panel_assemble applied: N={} size={} encap={}",
                  in.cell_count, in.cell_size_mm, in.encapsulant);

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

}  // namespace koocadcam::skill::solar_panel_assemble
