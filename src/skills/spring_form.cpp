// @lat: [[engine/skills#spring_form]]

#include "spring_form.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::spring_form {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wire_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form wire_dia_mm must be > 0 (got " +
              std::to_string(in.wire_dia_mm) + ")");
    }
    if (in.n_turns <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form n_turns must be > 0 (got " +
              std::to_string(in.n_turns) + ")");
    }
    if (in.coil_pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form coil_pitch_mm must be > 0 (got " +
              std::to_string(in.coil_pitch_mm) + ")");
    }
    if (in.free_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form free_length_mm must be > 0 (got " +
              std::to_string(in.free_length_mm) + ")");
    }
    if (in.mean_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form mean_dia_mm must be > 0 (got " +
              std::to_string(in.mean_dia_mm) + ")");
    }

    // Solid-height check: pitch must exceed wire diameter (else coils touch).
    if (in.coil_pitch_mm > 0.0 && in.wire_dia_mm > 0.0 &&
        in.coil_pitch_mm <= in.wire_dia_mm) {
        r.add("DFM-SPR-SOLID", "error",
              "spring_form coil_pitch " + std::to_string(in.coil_pitch_mm) +
              " mm ≤ wire_dia " + std::to_string(in.wire_dia_mm) +
              " mm — coils would touch in unloaded state");
    }

    // Spring index sanity.
    if (in.wire_dia_mm > 0.0 && in.mean_dia_mm > 0.0) {
        const double C = in.mean_dia_mm / in.wire_dia_mm;
        if (C < 4.0 || C > 12.0) {
            r.add("DFM-SPR-INDEX", "warning",
                  "spring_form spring_index C = " + std::to_string(C) +
                  " outside typical [4, 12] — stress / buckling risk");
        }
    }

    // Slenderness check.
    if (in.mean_dia_mm > 0.0 && in.free_length_mm > 0.0) {
        const double slenderness = in.free_length_mm / in.mean_dia_mm;
        if (slenderness > 4.0) {
            r.add("DFM-SPR-LEN", "warning",
                  "spring_form slenderness L/D = " + std::to_string(slenderness) +
                  " > 4 — buckling risk under compression");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Pure metadata stamp; geometry passes through unchanged.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spring_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double springIndex = (in.wire_dia_mm > 0.0)
                               ? in.mean_dia_mm / in.wire_dia_mm : 0.0;
    const double solidHeight = in.n_turns * in.wire_dia_mm;

    // Approximate wire length used: π · D_mean · n_turns (per coil).
    const double wireLength = M_PI * in.mean_dia_mm * in.n_turns;

    json params = {
        { "coil_pitch_mm",   in.coil_pitch_mm },
        { "n_turns",         in.n_turns },
        { "wire_dia_mm",     in.wire_dia_mm },
        { "free_length_mm",  in.free_length_mm },
        { "mean_dia_mm",     in.mean_dia_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "coil_pitch_mm",    in.coil_pitch_mm },
        { "n_turns",          in.n_turns },
        { "wire_dia_mm",      in.wire_dia_mm },
        { "free_length_mm",   in.free_length_mm },
        { "mean_dia_mm",      in.mean_dia_mm },
        { "spring_index",     springIndex },
        { "solid_height_mm",  solidHeight },
        { "wire_length_mm",   wireLength },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "coiling_arbor";
    tooling.tool_dia_mm       = in.mean_dia_mm - in.wire_dia_mm;
    tooling.tool_length_mm    = in.free_length_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = std::max(5.0, in.n_turns * 0.5);
    tooling.extra = json{
        { "process",        "wire_coiling" },
        { "spring_index",   springIndex },
        { "solid_height_mm", solidHeight },
        { "wire_length_mm", wireLength },
        { "geometric_no_op", true },
        { "dfm_findings",   findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::spring_form applied: n={} pitch={} d={} L={}",
                  in.n_turns, in.coil_pitch_mm, in.wire_dia_mm, in.free_length_mm);

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

}  // namespace koocadcam::skill::spring_form
