// @lat: [[engine/skills#sled_test]]

#include "sled_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <set>
#include <string>

namespace koocadcam::skill::sled_test {

using nlohmann::json;

namespace {

bool isKnownClass(const std::string& c)
{
    static const std::set<std::string> ok = {
        "crash", "shock", "launch", "generic"
    };
    return ok.count(c) > 0;
}

// Haversine pulse Δv ≈ (π/2) · g_peak · pulse_s (m/s)
double haversineDeltaV(double g_peak, double ms)
{
    const double g  = 9.81;                     // m/s^2
    const double t  = std::max(0.0, ms) * 1e-3; // s
    return (M_PI / 2.0) * g_peak * g * t;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pulse_g_peak <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pulse_g_peak must be > 0 (got " +
              std::to_string(in.pulse_g_peak) + ")");
    } else if (in.pulse_g_peak > 1000.0) {
        r.add("DFM-SLED-G", "info",
              std::string(kSkillId) + ": pulse_g_peak " +
              std::to_string(in.pulse_g_peak) +
              " > 1000g — likely beyond automotive sled fixture envelope");
    }

    if (in.duration_ms <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": duration_ms must be > 0 (got " +
              std::to_string(in.duration_ms) + ")");
    } else {
        if (in.duration_ms < 1.0) {
            r.add("DFM-SLED-DURATION", "info",
                  std::string(kSkillId) + ": duration_ms " +
                  std::to_string(in.duration_ms) +
                  " < 1 — pulse too narrow, consider shock_test fixture");
        }
        if (in.duration_ms > 500.0) {
            r.add("DFM-SLED-DURATION", "info",
                  std::string(kSkillId) + ": duration_ms " +
                  std::to_string(in.duration_ms) +
                  " > 500 — outside typical haversine pulse band");
        }
    }

    if (!isKnownClass(in.impact_class)) {
        r.add("DFM-SLED-CLASS", "info",
              std::string(kSkillId) + ": impact_class '" + in.impact_class +
              "' unknown — treated as 'generic'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sled_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string klass = isKnownClass(in.impact_class) ? in.impact_class : "generic";
    const double      dv    = haversineDeltaV(in.pulse_g_peak, in.duration_ms);

    json params = {
        { "pulse_g_peak", in.pulse_g_peak },
        { "duration_ms",  in.duration_ms  },
        { "impact_class", in.impact_class },
    };
    json pattern = {
        { "kind",             kSkillId         },
        { "pulse_g_peak",     in.pulse_g_peak  },
        { "duration_ms",      in.duration_ms   },
        { "impact_class",     klass            },
        { "delta_v_m_s",      dv               },
        { "pulse_shape",      "haversine"      },
        { "test_standard",    "SAE_J211"       },
        { "geometry_changed", false            },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "sled_test_fixture";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "aluminum_t_slot_table";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 60.0;     // mount + fire pulse + observe
    tooling.extra = {
        { "process",      "destructive_sled_pulse" },
        { "standard",     "SAE_J211"               },
        { "impact_class", klass                    },
        { "pulse_shape",  "haversine"              },
        { "delta_v_m_s",  dv                       },
        { "dfm_findings", findings                 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::sled_test applied: G={} dur={}ms class={} dv={}m/s",
                  in.pulse_g_peak, in.duration_ms, klass, dv);

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

}  // namespace koocadcam::skill::sled_test
