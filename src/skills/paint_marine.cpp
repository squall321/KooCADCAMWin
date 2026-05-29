// @lat: [[engine/skills#paint_marine]]

#include "paint_marine.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::paint_marine {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.coat_system.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": coat_system must be non-empty");
    }

    if (in.dft_um < 50.0 || in.dft_um > 1500.0) {
        r.add("DFM-PAINT-DFT", "error",
              std::string(kSkillId) + ": dft_um " +
              std::to_string(in.dft_um) +
              " outside [50, 1500] μm — outside marine NORSOK M-501 DFT range");
    }

    if (in.area_m2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": area_m2 must be > 0 (got " +
              std::to_string(in.area_m2) + ")");
    }

    if (in.dft_um > 800.0) {
        r.add("DFM-PAINT-DFT", "info",
              std::string(kSkillId) + ": dft_um " +
              std::to_string(in.dft_um) +
              " > 800 μm — apply as stripe-coat + extended cure between coats");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "paint_marine DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Coating volume (mm3) = area (m² → mm²) × dft (μm → mm)
    const double coatingVolumeMm3 =
        (in.area_m2 * 1.0e6) * (in.dft_um * 1.0e-3);

    json params = {
        { "coat_system", in.coat_system },
        { "dft_um",      in.dft_um },
        { "area_m2",     in.area_m2 },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "coat_system",          in.coat_system },
        { "dft_um",               in.dft_um },
        { "area_m2",              in.area_m2 },
        { "coating_volume_mm3",   coatingVolumeMm3 },
        { "geometry_changed",     false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "airless_spray_rig";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Coating ADDS material — record as negative removal (mass-balance,
    // mirrors concrete_pour).
    tooling.stock_removed_mm3 = -coatingVolumeMm3;
    // Airless spray ~ 200 m²/h per crew; add 12 h cure margin per coat pass.
    tooling.est_cycle_time_s  =
        (in.area_m2 / 200.0) * 3600.0 + 12.0 * 3600.0;
    tooling.extra = {
        { "process",          "marine_protective_coating" },
        { "coat_system",      in.coat_system },
        { "dft_um",           in.dft_um },
        { "area_m2",          in.area_m2 },
        { "code_reference",   "NORSOK_M-501" },
        { "process_category", "shipbuilding_coating" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::paint_marine applied: system={} dft={}um area={}m2",
        in.coat_system, in.dft_um, in.area_m2);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

}  // namespace koocadcam::skill::paint_marine
