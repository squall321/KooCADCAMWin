// @lat: [[engine/skills#xray_weld_inspect]]

#include "xray_weld_inspect.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::xray_weld_inspect {

using nlohmann::json;

namespace {

bool isAllowedDefectClass(const std::string& c)
{
    return c == "D1" || c == "D2" || c == "D3";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.weld_id.empty()) {
        r.add("DFM-INPUT", "error",
              "xray_weld_inspect weld_id must be non-empty");
    }
    if (in.kv <= 20.0 || in.kv > 600.0) {
        r.add("DFM-INPUT", "error",
              "xray_weld_inspect kv must be in (20, 600] (got " +
              std::to_string(in.kv) + ")");
    }
    if (in.exposure_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              "xray_weld_inspect exposure_min must be > 0 (got " +
              std::to_string(in.exposure_min) + ")");
    }
    if (!isAllowedDefectClass(in.defect_class)) {
        r.add("DFM-NDT-CLASS", "error",
              "xray_weld_inspect defect_class '" + in.defect_class +
              "' not in {D1, D2, D3}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "xray_weld_inspect DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "weld_id",      in.weld_id },
        { "kv",           in.kv },
        { "exposure_min", in.exposure_min },
        { "defect_class", in.defect_class },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "weld_id",          in.weld_id },
        { "kv",               in.kv },
        { "exposure_min",     in.exposure_min },
        { "defect_class",     in.defect_class },
        { "code_reference",   "AWS_D1" },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "xray_radiograph";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tungsten_target";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.exposure_min * 60.0;
    tooling.extra = {
        { "process",        "ndt_radiographic" },
        { "weld_id",        in.weld_id },
        { "kv",             in.kv },
        { "exposure_min",   in.exposure_min },
        { "defect_class",   in.defect_class },
        { "code",           "AWS_D1" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::xray_weld_inspect applied: weld={} kv={} t={} class={}",
                  in.weld_id, in.kv, in.exposure_min, in.defect_class);

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

}  // namespace koocadcam::skill::xray_weld_inspect
