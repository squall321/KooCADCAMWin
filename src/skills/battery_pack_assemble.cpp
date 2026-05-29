// @lat: [[engine/skills#battery_pack_assemble]]

#include "battery_pack_assemble.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::battery_pack_assemble {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.module_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": module_count must be > 0 (got " +
              std::to_string(in.module_count) + ")");
    }

    if (in.total_kwh <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": total_kwh must be > 0 (got " +
              std::to_string(in.total_kwh) + ")");
    }

    if (in.cooling_type != "liquid" && in.cooling_type != "air") {
        r.add("DFM-PACK-COOL", "error",
              std::string(kSkillId) + ": cooling_type '" + in.cooling_type +
              "' not in {liquid, air}");
    }

    if (in.module_count > 0 && in.total_kwh > 0.0) {
        const double per = in.total_kwh / static_cast<double>(in.module_count);
        if (per > 12.0) {
            r.add("DFM-PACK-KWH", "info",
                  std::string(kSkillId) + ": kWh per module " +
                  std::to_string(per) +
                  " > 12 — high energy density, verify thermal design");
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
        std::string msg = "battery_pack_assemble DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "module_count", in.module_count },
        { "total_kwh",    in.total_kwh },
        { "cooling_type", in.cooling_type },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "module_count",     in.module_count },
        { "total_kwh",        in.total_kwh },
        { "cooling_type",     in.cooling_type },
        { "kwh_per_module",
          in.module_count > 0
              ? in.total_kwh / static_cast<double>(in.module_count) : 0.0 },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = (in.cooling_type == "liquid")
                                    ? "liquid_cooled_pack_assembly_station"
                                    : "air_cooled_pack_assembly_station";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "aluminum_tray";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 45 s per module placement + 120 s for coolant connection (liquid only).
    tooling.est_cycle_time_s  = 45.0 * static_cast<double>(in.module_count)
                                + ((in.cooling_type == "liquid") ? 120.0 : 0.0);
    tooling.extra = {
        { "process",      "ev_battery_pack_assembly" },
        { "module_count", in.module_count },
        { "total_kwh",    in.total_kwh },
        { "cooling_type", in.cooling_type },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::battery_pack_assemble applied: modules={} kwh={} cooling={}",
                  in.module_count, in.total_kwh, in.cooling_type);

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

}  // namespace koocadcam::skill::battery_pack_assemble
