// @lat: [[engine/skills#composite_layup]]

#include "composite_layup.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace koocadcam::skill::composite_layup {

using nlohmann::json;

namespace {

// A layup is "symmetric" if orientations[i] == orientations[N-1-i] for all i.
bool isSymmetric(const std::vector<double>& orient)
{
    const std::size_t n = orient.size();
    for (std::size_t i = 0; i < n / 2; ++i) {
        if (std::abs(orient[i] - orient[n - 1 - i]) > 1e-6) return false;
    }
    return true;
}

// A layup is "balanced" if every +θ ply (0 < θ < 90) is matched by an equal
// count of −θ plies.  0°/90° plies do not need a partner.
bool isBalanced(const std::vector<double>& orient)
{
    std::vector<double> off;
    for (double a : orient) {
        if (std::abs(a) < 1e-6) continue;        // 0°
        if (std::abs(std::abs(a) - 90.0) < 1e-6) continue;  // 90°
        off.push_back(a);
    }
    for (double a : off) {
        if (a <= 0.0) continue;
        const auto cntPos = std::count_if(off.begin(), off.end(),
            [&](double x){ return std::abs(x - a) < 1e-6; });
        const auto cntNeg = std::count_if(off.begin(), off.end(),
            [&](double x){ return std::abs(x + a) < 1e-6; });
        if (cntPos != cntNeg) return false;
    }
    return true;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.ply_count < 1) {
        r.add("DFM-LAYUP-PLIES", "error",
              std::string(kSkillId) + ": ply_count must be ≥ 1 (got " +
              std::to_string(in.ply_count) + ")");
    }
    if (in.ply_thickness_mm < 0.05 || in.ply_thickness_mm > 0.5) {
        r.add("DFM-LAYUP-THICK", "error",
              std::string(kSkillId) + ": ply_thickness_mm " +
              std::to_string(in.ply_thickness_mm) +
              " outside typical CPT range [0.05, 0.5] mm");
    }
    if (static_cast<int>(in.orientation_deg.size()) != in.ply_count) {
        r.add("DFM-LAYUP-ORIENT", "error",
              std::string(kSkillId) + ": orientation_deg.size() " +
              std::to_string(in.orientation_deg.size()) +
              " must equal ply_count " + std::to_string(in.ply_count));
    }
    for (double a : in.orientation_deg) {
        if (a < -90.0 || a > 90.0) {
            r.add("DFM-LAYUP-RANGE", "error",
                  std::string(kSkillId) + ": orientation " +
                  std::to_string(a) + " deg outside [-90, 90]");
            break;
        }
    }
    if (in.matrix_material.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": matrix_material must be specified");
    }
    if (in.fiber_material.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": fiber_material must be specified");
    }

    // Symmetry / balance — info-only (out-of-plane laminates are valid).
    if (in.ply_count == static_cast<int>(in.orientation_deg.size())) {
        if (!isSymmetric(in.orientation_deg)) {
            r.add("DFM-LAYUP-SYM", "info",
                  std::string(kSkillId) + ": stack-up is not symmetric — "
                  "expect bending-extension coupling (B matrix ≠ 0)");
        }
        if (!isBalanced(in.orientation_deg)) {
            r.add("DFM-LAYUP-BAL", "info",
                  std::string(kSkillId) + ": stack-up is not balanced — "
                  "expect in-plane shear-extension coupling");
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
        std::string msg = "composite_layup DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const bool sym = isSymmetric(in.orientation_deg);
    const bool bal = isBalanced(in.orientation_deg);
    const double cpt = static_cast<double>(in.ply_count) * in.ply_thickness_mm;

    json params = {
        { "ply_count",         in.ply_count },
        { "ply_thickness_mm",  in.ply_thickness_mm },
        { "orientation_deg",   in.orientation_deg },
        { "matrix_material",   in.matrix_material },
        { "fiber_material",    in.fiber_material },
    };

    json pattern = {
        { "kind",              kSkillId },
        { "ply_count",         in.ply_count },
        { "ply_thickness_mm",  in.ply_thickness_mm },
        { "orientation_deg",   in.orientation_deg },
        { "matrix_material",   in.matrix_material },
        { "fiber_material",    in.fiber_material },
        { "symmetric",         sym },
        { "balanced",          bal },
        { "cpt_mm",            cpt },
        { "geometric_change",  false },
        { "process_category",  "composite" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "layup_table";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Manual layup hand-lay: ~ 3 min per ply on a small panel.
    tooling.est_cycle_time_s  = static_cast<double>(in.ply_count) * 180.0;
    tooling.extra = {
        { "matrix_material",   in.matrix_material },
        { "fiber_material",    in.fiber_material },
        { "ply_count",         in.ply_count },
        { "cpt_mm",            cpt },
        { "symmetric",         sym },
        { "balanced",          bal },
        { "dfm_findings",      findings },
        { "process_category",  "composite_layup" },
        { "geometric_no_op",   true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::composite_layup applied: plies={} cpt={} mm sym={} bal={}",
                  in.ply_count, cpt, sym, bal);

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
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::composite_layup
