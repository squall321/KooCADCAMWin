// @lat: [[engine/skills#peen_form]]

#include "peen_form.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::peen_form {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.peening_intensity <= 0.0) {
        r.add("DFM-INPUT", "error",
              "peen_form peening_intensity must be > 0 (got " +
              std::to_string(in.peening_intensity) + ")");
    }
    if (in.target_curvature_mm_inv <= 0.0) {
        r.add("DFM-INPUT", "error",
              "peen_form target_curvature_mm_inv must be > 0 (got " +
              std::to_string(in.target_curvature_mm_inv) + ")");
    }
    if (in.coverage_pct <= 0.0) {
        r.add("DFM-INPUT", "error",
              "peen_form coverage_pct must be > 0 (got " +
              std::to_string(in.coverage_pct) + ")");
    }

    // Industrial intensity range.
    if (in.peening_intensity > 0.0 &&
        (in.peening_intensity < 0.05 || in.peening_intensity > 0.50)) {
        r.add("DFM-PEEN-INTENS", "warning",
              "peen_form intensity " + std::to_string(in.peening_intensity) +
              " mm outside typical [0.05, 0.50] mm Almen-A range");
    }

    // Curvature capability check (tightest practical R ≈ 100 mm → 0.01 mm⁻¹).
    if (in.target_curvature_mm_inv > 0.01) {
        r.add("DFM-PEEN-CURV", "warning",
              "peen_form curvature " +
              std::to_string(in.target_curvature_mm_inv) +
              " mm⁻¹ (R = " +
              std::to_string(1.0 / in.target_curvature_mm_inv) +
              " mm) — tight curvature near peen-forming capability limit");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Geometry pass-through.  Stamp signature only.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "peen_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double radius = (in.target_curvature_mm_inv > 0.0)
                          ? 1.0 / in.target_curvature_mm_inv : 0.0;

    json params = {
        { "peening_intensity",       in.peening_intensity },
        { "target_curvature_mm_inv", in.target_curvature_mm_inv },
        { "coverage_pct",            in.coverage_pct },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "peening_intensity",          in.peening_intensity },
        { "target_curvature_mm_inv",    in.target_curvature_mm_inv },
        { "target_radius_mm",           radius },
        { "coverage_pct",               in.coverage_pct },
        { "geometry_changed",           false },
        { "induces_compressive_stress", true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "peen_nozzle";
    tooling.tool_dia_mm       = 10.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tungsten_carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 30.0 + in.coverage_pct * 0.5;
    tooling.extra = json{
        { "process",                "shot_peen_forming" },
        { "almen_intensity_mm",     in.peening_intensity },
        { "coverage_pct",           in.coverage_pct },
        { "target_radius_mm",       radius },
        { "geometric_no_op",        true },
        { "dfm_findings",           findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::peen_form applied: intensity={} curvature={}",
                  in.peening_intensity, in.target_curvature_mm_inv);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pure metadata replay — geometry cannot reveal a prior peen-form pass.

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

}  // namespace koocadcam::skill::peen_form
