// @lat: [[engine/skills#compression_test]]

#include "compression_test.hpp"

#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace koocadcam::skill::compression_test {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.peak_load_kn <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": peak_load_kn must be > 0 (got " +
              std::to_string(in.peak_load_kn) + ")");
    } else if (in.peak_load_kn > 1000.0) {
        r.add("DFM-COMP-LOAD", "info",
              std::string(kSkillId) + ": peak_load_kn " +
              std::to_string(in.peak_load_kn) +
              " > 1000 — beyond typical bench compression machine range");
    }

    if (in.displacement_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": displacement_mm must be > 0 (got " +
              std::to_string(in.displacement_mm) + ")");
    }

    if (in.rate_mm_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": rate_mm_min must be > 0 (got " +
              std::to_string(in.rate_mm_min) + ")");
    } else {
        if (in.rate_mm_min < 0.001) {
            r.add("DFM-COMP-RATE", "info",
                  std::string(kSkillId) + ": rate_mm_min " +
                  std::to_string(in.rate_mm_min) +
                  " < 0.001 — quasi-static creep regime");
        }
        if (in.rate_mm_min > 500.0) {
            r.add("DFM-COMP-RATE", "info",
                  std::string(kSkillId) + ": rate_mm_min " +
                  std::to_string(in.rate_mm_min) +
                  " > 500 — dynamic regime, use drop_test or sled_test instead");
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
        std::string msg = "compression_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Rough compressive stress proxy: peak_load / largest face projected area.
    Bnd_Box bb;
    BRepBndLib::Add(wp.shape(), bb);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    const double dx = std::max(0.001, xmax - xmin);
    const double dy = std::max(0.001, ymax - ymin);
    const double area_mm2 = std::max(0.001, dx * dy);
    // 1 kN / mm^2 == 1000 N / mm^2 == 1000 MPa.
    const double stressMpa = (in.peak_load_kn * 1000.0) / area_mm2;

    json params = {
        { "peak_load_kn",    in.peak_load_kn    },
        { "displacement_mm", in.displacement_mm },
        { "rate_mm_min",     in.rate_mm_min     },
    };
    json pattern = {
        { "kind",                          kSkillId            },
        { "peak_load_kn",                  in.peak_load_kn     },
        { "displacement_mm",               in.displacement_mm  },
        { "rate_mm_min",                   in.rate_mm_min      },
        { "nominal_area_mm2",              area_mm2            },
        { "compressive_stress_proxy_mpa",  stressMpa           },
        { "test_standard",                 "ASTM_E9"           },
        { "geometry_changed",              false               },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "universal_testing_machine";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.displacement_mm;
    tooling.tool_material     = "hardened_steel_platen";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // est cycle: displacement / rate (mm / (mm/min)) → min → s
    tooling.est_cycle_time_s  = (in.displacement_mm / in.rate_mm_min) * 60.0;
    tooling.extra = {
        { "process",                      "destructive_compression_test" },
        { "standard",                     "ASTM_E9"                      },
        { "peak_load_kn",                 in.peak_load_kn                },
        { "rate_mm_min",                  in.rate_mm_min                 },
        { "compressive_stress_proxy_mpa", stressMpa                      },
        { "dfm_findings",                 findings                       },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::compression_test applied: F={}kN d={}mm rate={}mm/min sigma~{}MPa",
                  in.peak_load_kn, in.displacement_mm, in.rate_mm_min, stressMpa);

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

}  // namespace koocadcam::skill::compression_test
