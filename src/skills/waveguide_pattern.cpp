// @lat: [[engine/skills#waveguide_pattern]]

#include "waveguide_pattern.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace koocadcam::skill::waveguide_pattern {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mask_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": mask_id must not be empty");
    }

    if (in.etch_depth_nm < 50.0 || in.etch_depth_nm > 5000.0) {
        r.add("DFM-WG-DEPTH", "error",
              std::string(kSkillId) + ": etch_depth_nm " +
              std::to_string(in.etch_depth_nm) +
              " not in [50, 5000] nm");
    }

    if (in.line_width_nm < 100.0 || in.line_width_nm > 10000.0) {
        r.add("DFM-WG-WIDTH", "error",
              std::string(kSkillId) + ": line_width_nm " +
              std::to_string(in.line_width_nm) +
              " not in [100, 10000] nm");
    }

    if (in.etch_depth_nm > 0.0 && in.line_width_nm > 0.0) {
        const double aspect = in.etch_depth_nm / in.line_width_nm;
        if (aspect > 10.0) {
            r.add("DFM-WG-ASPECT", "info",
                  std::string(kSkillId) + ": aspect_ratio " +
                  std::to_string(aspect) +
                  " > 10 — beyond typical DRIE notching-free limit");
        }
    }

    const bool knownProcess =
        (in.process == "RIE" || in.process == "DRIE" || in.process == "ICP");
    if (!knownProcess) {
        r.add("DFM-WG-PROCESS", "info",
              std::string(kSkillId) + ": process '" + in.process +
              "' unrecognised — expected RIE | DRIE | ICP");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "waveguide_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double aspect = (in.line_width_nm > 0.0)
                        ? in.etch_depth_nm / in.line_width_nm
                        : 0.0;

    json params = {
        { "mask_id",       in.mask_id },
        { "etch_depth_nm", in.etch_depth_nm },
        { "line_width_nm", in.line_width_nm },
        { "process",       in.process },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "mask_id",          in.mask_id },
        { "etch_depth_nm",    in.etch_depth_nm },
        { "line_width_nm",    in.line_width_nm },
        { "process",          in.process },
        { "aspect_ratio",     aspect },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = in.process + "_etcher";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "fluorinated_plasma";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;       // nm-scale removal, negligible at mm³
    // ICP etch rate ~ 300 nm/min for SOI Si; cycle = depth/rate + 60 s pump.
    const double etch_rate_nm_per_min = (in.process == "DRIE") ? 1500.0
                                       : (in.process == "RIE")  ? 100.0
                                                                : 300.0;
    tooling.est_cycle_time_s =
        (in.etch_depth_nm / etch_rate_nm_per_min) * 60.0 + 60.0;
    tooling.extra = {
        { "process",       "photonic_waveguide_etch" },
        { "etch_process",  in.process },
        { "mask_id",       in.mask_id },
        { "etch_depth_nm", in.etch_depth_nm },
        { "line_width_nm", in.line_width_nm },
        { "aspect_ratio",  aspect },
        { "dfm_findings",  findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::waveguide_pattern applied: mask={} depth={}nm width={}nm",
                  in.mask_id, in.etch_depth_nm, in.line_width_nm);

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

}  // namespace koocadcam::skill::waveguide_pattern
