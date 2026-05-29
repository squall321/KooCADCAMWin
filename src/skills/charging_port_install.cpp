// @lat: [[engine/skills#charging_port_install]]

#include "charging_port_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::charging_port_install {

using nlohmann::json;

namespace {

// Per-standard peak rating ceiling (kW).
double portCeilingKw(const std::string& type)
{
    if (type == "CCS")     return 350.0;   // CCS2 up to 350 kW
    if (type == "CHAdeMO") return 400.0;   // CHAdeMO 3.0 up to 400 kW (rare)
    if (type == "Type2")   return 43.0;    // AC Type 2 up to 43 kW (3-phase)
    if (type == "NACS")    return 250.0;   // Tesla NACS up to 250 kW
    return 0.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const bool typeOk =
        in.port_type == "CCS"     || in.port_type == "CHAdeMO" ||
        in.port_type == "Type2"   || in.port_type == "NACS";
    if (!typeOk) {
        r.add("DFM-PORT-TYPE", "error",
              std::string(kSkillId) + ": port_type '" + in.port_type +
              "' not in {CCS, CHAdeMO, Type2, NACS}");
    }

    if (in.max_kw <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": max_kw must be > 0 (got " +
              std::to_string(in.max_kw) + ")");
    } else if (typeOk) {
        const double ceiling = portCeilingKw(in.port_type);
        if (in.max_kw > ceiling) {
            r.add("DFM-PORT-KW", "error",
                  std::string(kSkillId) + ": max_kw " +
                  std::to_string(in.max_kw) +
                  " > standard ceiling " + std::to_string(ceiling) +
                  " kW for '" + in.port_type + "'");
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
        std::string msg = "charging_port_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "port_type", in.port_type },
        { "max_kw",    in.max_kw },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "port_type",        in.port_type },
        { "max_kw",           in.max_kw },
        { "standard_ceiling", portCeilingKw(in.port_type) },
        { "current_type",
          (in.port_type == "Type2") ? std::string("AC") : std::string("DC") },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "charging_port_install_station";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "thermoplastic_housing";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 60 s mechanical seat + 30 s torque + interlock probe.
    tooling.est_cycle_time_s  = 90.0;
    tooling.extra = {
        { "process",   "ev_charging_port_install" },
        { "port_type", in.port_type },
        { "max_kw",    in.max_kw },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::charging_port_install applied: type={} max_kw={}",
                  in.port_type, in.max_kw);

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

}  // namespace koocadcam::skill::charging_port_install
