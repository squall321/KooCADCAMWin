// @lat: [[engine/skills#statistical_process_ctl]]

#include "statistical_process_ctl.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::statistical_process_ctl {

using nlohmann::json;

// ── Capability index helpers ─────────────────────────────────────────────
//
//          USL − LSL                       ⎛ USL − μ     μ − LSL ⎞
//   Cp  = ───────────             Cpk = min⎜ ───────  ,  ─────── ⎟
//             6 σ                          ⎝   3 σ          3 σ   ⎠
//
// Cp is symmetric in (USL − μ) vs (μ − LSL); Cpk penalises off-centring.
// Identity: Cpk == Cp iff μ == (USL + LSL) / 2 (the mid-point).

std::pair<double, double> computeCpCpk(double mean, double sigma,
                                       double USL, double LSL)
{
    if (sigma <= 0.0 || !(USL > LSL)) {
        return { 0.0, 0.0 };
    }
    const double cp  = (USL - LSL) / (6.0 * sigma);
    const double cpu = (USL - mean) / (3.0 * sigma);
    const double cpl = (mean - LSL) / (3.0 * sigma);
    const double cpk = std::min(cpu, cpl);
    return { cp, cpk };
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.sigma <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sigma must be > 0 (got " +
              std::to_string(in.sigma) + ")");
    }
    if (!(in.USL > in.LSL)) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": USL (" + std::to_string(in.USL) +
              ") must be > LSL (" + std::to_string(in.LSL) + ")");
    }
    if (in.sample_size < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sample_size must be >= 1 (got " +
              std::to_string(in.sample_size) + ")");
    }

    // If basic ranges are sane, compute Cp/Cpk for info-level checks.
    if (in.sigma > 0.0 && in.USL > in.LSL) {
        const auto [cp, cpk] = computeCpCpk(in.mean, in.sigma, in.USL, in.LSL);
        if (cpk < 1.0) {
            r.add("DFM-SPC-INCAPABLE", "info",
                  std::string(kSkillId) + ": Cpk " + std::to_string(cpk) +
                  " < 1.0 — process not capable; corrective action required");
        }
        if (cp > 0.0 && std::fabs(cp - cpk) > 0.1 * cp) {
            r.add("DFM-SPC-OFFCENTRE", "info",
                  std::string(kSkillId) + ": |Cp − Cpk| > 0.1·Cp — process "
                  "mean is off-centre between spec limits; re-centre tooling");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

std::string verdictBucket(double cpk)
{
    if (cpk < 1.00) return "incapable";
    if (cpk < 1.33) return "marginal";
    if (cpk < 1.67) return "capable";
    return "highly_capable";
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "statistical_process_ctl DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto [cp, cpk] = computeCpCpk(in.mean, in.sigma, in.USL, in.LSL);
    const std::string verdict = verdictBucket(cpk);

    json params = {
        { "characteristic", in.characteristic },
        { "mean",           in.mean },
        { "sigma",          in.sigma },
        { "USL",            in.USL },
        { "LSL",            in.LSL },
        { "sample_size",    in.sample_size },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_inspection_only",   true },
        { "geometric_change",     false },
        { "characteristic",       in.characteristic },
        { "mean",                 in.mean },
        { "sigma",                in.sigma },
        { "USL",                  in.USL },
        { "LSL",                  in.LSL },
        { "cp",                   cp },
        { "cpk",                  cpk },
        { "sample_size",          in.sample_size },
        { "verdict",              verdict },
        { "inspection_method",    "spc_capability_study" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "spc_record";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Capability study compute time — small overhead per sample.
    tooling.est_cycle_time_s  =
        15.0 * static_cast<double>(in.sample_size);
    tooling.extra["inspection_type"] = "spc_capability";
    tooling.extra["cp"]              = cp;
    tooling.extra["cpk"]             = cpk;
    tooling.extra["verdict"]         = verdict;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::statistical_process_ctl applied: mean={} sigma={} "
        "USL={} LSL={} cp={} cpk={}",
        in.mean, in.sigma, in.USL, in.LSL, cp, cpk);

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

}  // namespace koocadcam::skill::statistical_process_ctl
