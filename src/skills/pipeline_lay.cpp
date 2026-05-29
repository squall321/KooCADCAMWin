// @lat: [[engine/skills#pipeline_lay]]

#include "pipeline_lay.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::pipeline_lay {

using nlohmann::json;

namespace {

bool isKnownTerrain(const std::string& t)
{
    return t == "flat" || t == "rolling" || t == "mountain"
        || t == "subsea" || t == "swamp";
}

double layRateKmPerDay(const std::string& terrain)
{
    if (terrain == "flat")     return 3.0;
    if (terrain == "rolling")  return 2.0;
    if (terrain == "mountain") return 0.8;
    if (terrain == "subsea")   return 1.5;
    if (terrain == "swamp")    return 1.0;
    return 1.5;  // default
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.pipe_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pipe_dia_mm must be > 0 (got " +
              std::to_string(in.pipe_dia_mm) + ")");
    } else if (in.pipe_dia_mm < 60.3 || in.pipe_dia_mm > 1219.0) {
        r.add("DFM-PL-DIA", "info",
              std::string(kSkillId) + ": pipe_dia_mm " +
              std::to_string(in.pipe_dia_mm) +
              " outside typical 2\"–48\" line-pipe range");
    }

    if (in.wall_thick_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": wall_thick_mm must be > 0 (got " +
              std::to_string(in.wall_thick_mm) + ")");
    } else if (in.pipe_dia_mm > 0.0) {
        if (in.wall_thick_mm >= in.pipe_dia_mm / 2.0) {
            r.add("DFM-PL-WALL", "error",
                  std::string(kSkillId) + ": wall_thick_mm " +
                  std::to_string(in.wall_thick_mm) +
                  " ≥ pipe_dia_mm/2 — wall consumes entire bore");
        }
        const double dOverT = in.pipe_dia_mm / in.wall_thick_mm;
        if (dOverT > 200.0) {
            r.add("DFM-PL-DR", "info",
                  std::string(kSkillId) + ": D/t " + std::to_string(dOverT) +
                  " > 200 — very thin-wall, verify collapse pressure rating");
        }
    }

    if (in.length_km <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": length_km must be > 0 (got " +
              std::to_string(in.length_km) + ")");
    }

    if (in.terrain_class.empty()) {
        r.add("DFM-INPUT", "warning",
              std::string(kSkillId) + ": terrain_class empty — defaulting to 'rolling'");
    } else if (!isKnownTerrain(in.terrain_class)) {
        r.add("DFM-PL-TERRAIN", "info",
              std::string(kSkillId) + ": terrain_class '" + in.terrain_class +
              "' not in {flat,rolling,mountain,subsea,swamp} — using default lay rate");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pipeline_lay DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string terrain = in.terrain_class.empty()
                                    ? std::string("rolling")
                                    : in.terrain_class;
    const double rate = layRateKmPerDay(terrain);
    const double durationDays = (rate > 0.0) ? in.length_km / rate : 0.0;

    json params = {
        { "pipe_dia_mm",    in.pipe_dia_mm },
        { "wall_thick_mm",  in.wall_thick_mm },
        { "length_km",      in.length_km },
        { "terrain_class",  terrain },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "pipe_dia_mm",      in.pipe_dia_mm },
        { "wall_thick_mm",    in.wall_thick_mm },
        { "length_km",        in.length_km },
        { "terrain_class",    terrain },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "pipelayer_spread";
    tooling.tool_dia_mm       = in.pipe_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "carbon_steel_x65";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle time in seconds (planning estimate, days × 86400).
    tooling.est_cycle_time_s  = durationDays * 86400.0;
    tooling.extra = {
        { "process",           "oilgas_pipeline_lay" },
        { "pipe_dia_mm",       in.pipe_dia_mm },
        { "wall_thick_mm",     in.wall_thick_mm },
        { "length_km",         in.length_km },
        { "terrain_class",     terrain },
        { "lay_rate_km_day",   rate },
        { "duration_days",     durationDays },
        { "process_category",  "linear_construction" },
        { "geometric_no_op",   true },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::pipeline_lay applied: dia={}mm wall={}mm length={}km terrain={}",
        in.pipe_dia_mm, in.wall_thick_mm, in.length_km, terrain);

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

}  // namespace koocadcam::skill::pipeline_lay
