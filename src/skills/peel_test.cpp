// @lat: [[engine/skills#peel_test]]

#include "peel_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <set>
#include <string>

namespace koocadcam::skill::peel_test {

using nlohmann::json;

namespace {

std::string bondQuality(double force_n_mm)
{
    if (force_n_mm < 0.5)  return "weak";
    if (force_n_mm > 5.0)  return "strong";
    return "nominal";
}

bool isKnownAdhesive(const std::string& a)
{
    static const std::set<std::string> ok = {
        "PSA", "epoxy", "cyanoacrylate", "silicone", "acrylic", "polyurethane"
    };
    return ok.count(a) > 0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.peel_angle_deg <= 0.0 || in.peel_angle_deg > 180.0) {
        r.add("DFM-PEEL-ANGLE", "error",
              std::string(kSkillId) + ": peel_angle_deg must be in (0, 180] (got " +
              std::to_string(in.peel_angle_deg) + ")");
    }

    if (in.peel_force_n_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": peel_force_n_mm must be > 0 (got " +
              std::to_string(in.peel_force_n_mm) + ")");
    } else if (in.peel_force_n_mm > 100.0) {
        r.add("DFM-PEEL-FORCE", "info",
              std::string(kSkillId) + ": peel_force_n_mm " +
              std::to_string(in.peel_force_n_mm) +
              " > 100 — beyond typical peel adhesion regime");
    }

    if (in.adhesive_type.empty()) {
        r.add("DFM-INPUT", "warning",
              std::string(kSkillId) + ": adhesive_type empty — defaulting to PSA");
    } else if (!isKnownAdhesive(in.adhesive_type)) {
        r.add("DFM-PEEL-ADHESIVE", "info",
              std::string(kSkillId) + ": adhesive_type '" + in.adhesive_type +
              "' not in known set — proceeding with recorded value");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "peel_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string adhesive = in.adhesive_type.empty() ? "PSA" : in.adhesive_type;
    const std::string quality  = bondQuality(in.peel_force_n_mm);

    json params = {
        { "peel_angle_deg",  in.peel_angle_deg  },
        { "peel_force_n_mm", in.peel_force_n_mm },
        { "adhesive_type",   in.adhesive_type   },
    };
    json pattern = {
        { "kind",             kSkillId            },
        { "peel_angle_deg",   in.peel_angle_deg   },
        { "peel_force_n_mm",  in.peel_force_n_mm  },
        { "adhesive_type",    adhesive            },
        { "bond_quality",     quality             },
        { "test_standard",    in.peel_angle_deg <= 90.0 ? "ASTM_D903" : "ASTM_D1876" },
        { "geometry_changed", false               },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "peel_test_grip";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "knurled_steel_grip";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 60.0;
    tooling.extra = {
        { "process",        "destructive_peel_test" },
        { "standard",       in.peel_angle_deg <= 90.0 ? "ASTM_D903" : "ASTM_D1876" },
        { "adhesive_type",  adhesive                },
        { "bond_quality",   quality                 },
        { "dfm_findings",   findings                },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::peel_test applied: angle={}deg F'={}N/mm adhesive={} quality={}",
                  in.peel_angle_deg, in.peel_force_n_mm, adhesive, quality);

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

}  // namespace koocadcam::skill::peel_test
