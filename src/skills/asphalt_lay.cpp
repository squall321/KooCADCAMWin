// @lat: [[engine/skills#asphalt_lay]]

#include "asphalt_lay.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::asphalt_lay {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.mix_class != "binder" && in.mix_class != "wearing") {
        r.add("DFM-ASPH-CLASS", "error",
              std::string(kSkillId) + ": mix_class '" + in.mix_class +
              "' invalid — must be 'binder' or 'wearing'");
    }

    if (in.lay_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": lay_thickness_mm must be > 0 (got " +
              std::to_string(in.lay_thickness_mm) + ")");
    }

    if (in.area_m2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": area_m2 must be > 0 (got " +
              std::to_string(in.area_m2) + ")");
    }

    if (in.mix_class == "wearing" && in.lay_thickness_mm > 0.0
        && in.lay_thickness_mm < 25.0) {
        r.add("DFM-ASPH-THK", "info",
              std::string(kSkillId) + ": wearing course thickness " +
              std::to_string(in.lay_thickness_mm) +
              " mm < 25 — thin overlay; verify aggregate top size compatibility");
    }

    if (in.lay_thickness_mm > 100.0) {
        r.add("DFM-ASPH-THK", "info",
              std::string(kSkillId) + ": lay_thickness_mm " +
              std::to_string(in.lay_thickness_mm) +
              " > 100 — recommend splitting into two compacted lifts");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "asphalt_lay DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "mix_class",        in.mix_class },
        { "lay_thickness_mm", in.lay_thickness_mm },
        { "area_m2",          in.area_m2 },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "mix_class",         in.mix_class },
        { "lay_thickness_mm",  in.lay_thickness_mm },
        { "area_m2",           in.area_m2 },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "asphalt_paver";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Paver typical rate ~ 600 m2/h (paving + breakdown roller); add 30 min
    // intermediate / finish rolling overhead.
    tooling.est_cycle_time_s  = (in.area_m2 / 600.0) * 3600.0 + 1800.0;
    tooling.extra = {
        { "process",          "asphalt_paving" },
        { "mix_class",        in.mix_class },
        { "lay_thickness_mm", in.lay_thickness_mm },
        { "area_m2",          in.area_m2 },
        { "code_reference",   "AASHTO_M323" },
        { "process_category", "civil_road_asphalt" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::asphalt_lay applied: class={} thk={}mm area={}m2",
        in.mix_class, in.lay_thickness_mm, in.area_m2);

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

}  // namespace koocadcam::skill::asphalt_lay
