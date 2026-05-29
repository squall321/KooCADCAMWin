// @lat: [[engine/skills#drop_test]]

#include "drop_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <set>
#include <string>

namespace koocadcam::skill::drop_test {

using nlohmann::json;

namespace {

bool isKnownAxis(const std::string& a)
{
    static const std::set<std::string> ok = {
        "+X", "-X", "+Y", "-Y", "+Z", "-Z"
    };
    return ok.count(a) > 0;
}

bool isKnownSurface(const std::string& s)
{
    static const std::set<std::string> ok = {
        "concrete", "steel", "wood", "rubber"
    };
    return ok.count(s) > 0;
}

// Rough rigid-body peak-G estimate.  Real value depends on damping and
// contact stiffness; this is a planning aid only.
double estimatePeakG(double height_mm, const std::string& surface)
{
    // Base: free-fall velocity v = sqrt(2 g h); stop distance models impact.
    const double g     = 9810.0;                       // mm/s^2
    const double h     = std::max(height_mm, 1.0);
    const double v     = std::sqrt(2.0 * g * h);       // mm/s
    double stop_mm     = 1.0;                          // concrete default
    if (surface == "steel")   stop_mm = 0.5;
    if (surface == "wood")    stop_mm = 3.0;
    if (surface == "rubber")  stop_mm = 10.0;
    // peak acceleration a ≈ v^2 / (2·stop)
    const double a = (v * v) / (2.0 * stop_mm);        // mm/s^2
    return a / g;                                       // in g
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.drop_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": drop_height_mm must be > 0 (got " +
              std::to_string(in.drop_height_mm) + ")");
    } else {
        if (in.drop_height_mm < 50.0) {
            r.add("DFM-DROP-HEIGHT", "info",
                  std::string(kSkillId) + ": drop_height_mm " +
                  std::to_string(in.drop_height_mm) +
                  " < 50 mm — below typical handling-drop floor");
        }
        if (in.drop_height_mm > 3000.0) {
            r.add("DFM-DROP-HEIGHT", "info",
                  std::string(kSkillId) + ": drop_height_mm " +
                  std::to_string(in.drop_height_mm) +
                  " > 3000 mm — exceeds ISO 2248 transit-drop maximum");
        }
    }

    if (!isKnownAxis(in.impact_axis)) {
        r.add("DFM-DROP-AXIS", "info",
              std::string(kSkillId) + ": impact_axis '" + in.impact_axis +
              "' not in {±X,±Y,±Z} — treated as -Z");
    }

    if (!isKnownSurface(in.surface_type)) {
        r.add("DFM-DROP-SURFACE", "info",
              std::string(kSkillId) + ": surface_type '" + in.surface_type +
              "' unknown — falling back to 'concrete' for peak-G estimate");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "drop_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string axis    = isKnownAxis(in.impact_axis)    ? in.impact_axis  : "-Z";
    const std::string surface = isKnownSurface(in.surface_type) ? in.surface_type : "concrete";
    const double      gPeak   = estimatePeakG(in.drop_height_mm, surface);

    json params = {
        { "drop_height_mm", in.drop_height_mm },
        { "impact_axis",    in.impact_axis    },
        { "surface_type",   in.surface_type   },
    };
    json pattern = {
        { "kind",             kSkillId        },
        { "drop_height_mm",   in.drop_height_mm },
        { "impact_axis",      axis            },
        { "surface_type",     surface         },
        { "expected_g_peak",  gPeak           },
        { "test_standard",    "ISO_2248"      },
        { "geometry_changed", false           },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "drop_test_rig";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.drop_height_mm;
    tooling.tool_material     = surface;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 30.0;     // rig set-up + drop + observe
    tooling.extra = {
        { "process",         "destructive_drop_test" },
        { "standard",        "ISO_2248"              },
        { "impact_axis",     axis                    },
        { "surface_type",    surface                 },
        { "expected_g_peak", gPeak                   },
        { "dfm_findings",    findings                },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone shape + material verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::drop_test applied: h={}mm axis={} surface={} ~{}g",
                  in.drop_height_mm, axis, surface, gPeak);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Drop test leaves no recoverable B-rep change.  Metadata replay only.

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

}  // namespace koocadcam::skill::drop_test
