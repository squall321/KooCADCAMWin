// @lat: [[engine/skills#wave_solder]]

#include "wave_solder.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>

namespace koocadcam::skill::wave_solder {

using nlohmann::json;

namespace {

// Catalog of solder alloys the wave-solder DFM gate recognises.  Membership
// is INFO-only — the world has thousands of solder formulations; the goal is
// to surface unfamiliar names, not to block planning.
bool isKnownSolderAlloy(const std::string& a)
{
    static const char* kKnown[] = {
        "SAC305",      // Sn96.5/Ag3.0/Cu0.5  (lead-free, IPC J-STD-006)
        "SAC405",      // Sn95.5/Ag4.0/Cu0.5
        "Sn63Pb37",    // eutectic tin-lead
        "Sn60Pb40",
        "SN100C",      // Sn/Cu/Ni (lead-free, low-cost)
        "Sn99Cu1",
    };
    for (const char* k : kKnown) {
        if (a == k) return true;
    }
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────
//
// Process gates implement the wave-solder window agreed in the
// project_external_research log (260 °C ± 30 °C, 0.4 – 2.0 m/min).  Out-of-
// range temperature is an ERROR (board delamination / solidus violation);
// out-of-range conveyor speed is INFO (process tunable for bridging).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.solder_alloy.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": solder_alloy must not be empty");
    }
    if (in.temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": temp_c must be > 0 (got " +
              std::to_string(in.temp_c) + ")");
    } else {
        if (in.temp_c < 230.0) {
            r.add("DFM-WAVE-TEMP-LOW", "error",
                  std::string(kSkillId) + ": temp_c " +
                  std::to_string(in.temp_c) +
                  " °C below Sn-Pb solidus 230 °C — wave will not flow");
        } else if (in.temp_c > 290.0) {
            r.add("DFM-WAVE-TEMP-HIGH", "error",
                  std::string(kSkillId) + ": temp_c " +
                  std::to_string(in.temp_c) +
                  " °C above 290 °C — FR-4 delamination risk");
        }
    }

    if (in.conveyor_speed <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": conveyor_speed must be > 0 (got " +
              std::to_string(in.conveyor_speed) + ")");
    } else {
        if (in.conveyor_speed < 0.4) {
            r.add("DFM-WAVE-SPEED-LOW", "info",
                  std::string(kSkillId) + ": conveyor_speed " +
                  std::to_string(in.conveyor_speed) +
                  " m/min < 0.4 — long dwell, bridging risk");
        } else if (in.conveyor_speed > 2.0) {
            r.add("DFM-WAVE-SPEED-HIGH", "info",
                  std::string(kSkillId) + ": conveyor_speed " +
                  std::to_string(in.conveyor_speed) +
                  " m/min > 2.0 — short dwell, cold-joint risk");
        }
    }

    if (!in.solder_alloy.empty() && !isKnownSolderAlloy(in.solder_alloy)) {
        r.add("DFM-WAVE-ALLOY", "info",
              std::string(kSkillId) + ": solder_alloy '" + in.solder_alloy +
              "' not in known catalog — process plan will treat as custom");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Effective wave width ~25 mm (industry-typical lambda+chip dual wave
    // exposure zone).  dwell_time_s = 0.025 m / (conveyor_speed m/min) × 60.
    const double effective_wave_m = 0.025;
    const double dwell_time_s     = (in.conveyor_speed > 0.0)
        ? (effective_wave_m / in.conveyor_speed) * 60.0
        : 0.0;

    json params = {
        { "solder_alloy",   in.solder_alloy },
        { "temp_c",         in.temp_c },
        { "conveyor_speed", in.conveyor_speed },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "solder_alloy",     in.solder_alloy },
        { "temp_c",           in.temp_c },
        { "conveyor_speed",   in.conveyor_speed },
        { "dwell_time_s",     dwell_time_s },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "wave_solder_pot";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel_pot";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // additive (no removal)
    tooling.est_cycle_time_s  = std::max(dwell_time_s, 5.0);
    tooling.extra = {
        { "process",          "wave_solder" },
        { "process_category", "electronics_assembly" },
        { "solder_alloy",     in.solder_alloy },
        { "temp_c",           in.temp_c },
        { "conveyor_speed",   in.conveyor_speed },
        { "dwell_time_s",     dwell_time_s },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::wave_solder applied: alloy={} T={}°C v={} m/min dwell={}s",
                  in.solder_alloy, in.temp_c, in.conveyor_speed, dwell_time_s);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Wave-solder fillets are sub-millimetre and absent from the OCCT model.
// Recognize ONLY via metadata replay.

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

}  // namespace koocadcam::skill::wave_solder
