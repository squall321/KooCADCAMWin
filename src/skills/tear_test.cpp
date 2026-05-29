// @lat: [[engine/skills#tear_test]]

#include "tear_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>

namespace koocadcam::skill::tear_test {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tear_force_n <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": tear_force_n must be > 0 (got " +
              std::to_string(in.tear_force_n) + ")");
    } else if (in.tear_force_n > 10000.0) {
        r.add("DFM-TEAR-FORCE", "info",
              std::string(kSkillId) + ": tear_force_n " +
              std::to_string(in.tear_force_n) +
              " > 10 kN — beyond elastomer/film regime, prefer compression_test");
    }

    if (in.sample_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sample_thickness_mm must be > 0 (got " +
              std::to_string(in.sample_thickness_mm) + ")");
    } else {
        if (in.sample_thickness_mm < 0.05) {
            r.add("DFM-TEAR-THICKNESS", "info",
                  std::string(kSkillId) + ": sample_thickness_mm " +
                  std::to_string(in.sample_thickness_mm) +
                  " < 0.05 — foil regime, gauge fixture carefully");
        }
        if (in.sample_thickness_mm > 25.0) {
            r.add("DFM-TEAR-THICKNESS", "info",
                  std::string(kSkillId) + ": sample_thickness_mm " +
                  std::to_string(in.sample_thickness_mm) +
                  " > 25 — too thick for trouser-leg geometry");
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
        std::string msg = "tear_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double t = std::max(in.sample_thickness_mm, 1e-6);
    const double tearStrength_n_mm = in.tear_force_n / t;

    json params = {
        { "tear_force_n",        in.tear_force_n        },
        { "sample_thickness_mm", in.sample_thickness_mm },
    };
    json pattern = {
        { "kind",                 kSkillId               },
        { "tear_force_n",         in.tear_force_n        },
        { "sample_thickness_mm",  in.sample_thickness_mm },
        { "tear_strength_n_mm",   tearStrength_n_mm      },
        { "specimen_geometry",    "trouser_leg"          },
        { "test_standard",        "ASTM_D624"            },
        { "geometry_changed",     false                  },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "tensile_tear_grips";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "knurled_steel_grip";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 45.0;
    tooling.extra = {
        { "process",            "destructive_tear_test"   },
        { "standard",           "ASTM_D624"               },
        { "specimen_geometry",  "trouser_leg"             },
        { "tear_strength_n_mm", tearStrength_n_mm         },
        { "dfm_findings",       findings                  },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::tear_test applied: F={}N t={}mm S={}N/mm",
                  in.tear_force_n, in.sample_thickness_mm, tearStrength_n_mm);

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

}  // namespace koocadcam::skill::tear_test
