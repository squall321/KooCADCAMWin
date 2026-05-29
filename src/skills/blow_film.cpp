// @lat: [[engine/skills#blow_film]]

#include "blow_film.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>

namespace koocadcam::skill::blow_film {

using nlohmann::json;

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bubble_dia_mm <= 0.0) {
        r.add("DFM-BLOW-DIA", "error",
              std::string(kSkillId) + ": bubble_dia_mm must be > 0 (got " +
              std::to_string(in.bubble_dia_mm) + ")");
    } else if (in.bubble_dia_mm > 3000.0) {
        r.add("DFM-BLOW-DIA", "error",
              std::string(kSkillId) + ": bubble_dia_mm " +
              std::to_string(in.bubble_dia_mm) +
              " > 3000 — beyond commercial blown-film tower");
    }

    if (in.layflat_width_mm <= 0.0) {
        r.add("DFM-BLOW-LAYFLAT", "error",
              std::string(kSkillId) + ": layflat_width_mm must be > 0 (got " +
              std::to_string(in.layflat_width_mm) + ")");
    } else if (in.layflat_width_mm > 5000.0) {
        r.add("DFM-BLOW-LAYFLAT", "error",
              std::string(kSkillId) + ": layflat_width_mm " +
              std::to_string(in.layflat_width_mm) +
              " > 5000 — beyond collapsing-frame width");
    }

    if (in.bur < 1.5) {
        r.add("DFM-BLOW-BUR", "error",
              std::string(kSkillId) + ": bur " + std::to_string(in.bur) +
              " < 1.5 — insufficient transverse orientation, weak film");
    } else if (in.bur > 4.0) {
        r.add("DFM-BLOW-BUR", "error",
              std::string(kSkillId) + ": bur " + std::to_string(in.bur) +
              " > 4.0 — bubble instability / frost-line collapse risk");
    }

    // Layflat consistency: layflat ≈ π × dia / 2 within ±10 %
    if (in.bubble_dia_mm > 0.0 && in.layflat_width_mm > 0.0) {
        const double expected = kPi * in.bubble_dia_mm / 2.0;
        const double err = std::fabs(in.layflat_width_mm - expected) / expected;
        if (err > 0.10) {
            r.add("DFM-BLOW-CONSISTENCY", "info",
                  std::string(kSkillId) + ": layflat_width_mm " +
                  std::to_string(in.layflat_width_mm) +
                  " inconsistent with π × dia / 2 = " +
                  std::to_string(expected) + " (>10% deviation)");
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
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double expectedLayflat = kPi * in.bubble_dia_mm / 2.0;

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    json params = {
        { "bubble_dia_mm",     in.bubble_dia_mm },
        { "layflat_width_mm",  in.layflat_width_mm },
        { "bur",               in.bur },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "bubble_dia_mm",              in.bubble_dia_mm },
        { "layflat_width_mm",           in.layflat_width_mm },
        { "bur",                        in.bur },
        { "expected_layflat_mm",        expectedLayflat },
        { "geometry_changed",           false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "blown_film_annular_die";
    tooling.tool_dia_mm       = in.bubble_dia_mm / in.bur;  // die Ø ~ bubble Ø / BUR
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "chrome_plated_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 3600.0;
    tooling.extra = {
        { "process",            "blown_film_extrusion" },
        { "bubble_dia_mm",      in.bubble_dia_mm },
        { "layflat_width_mm",   in.layflat_width_mm },
        { "bur",                in.bur },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::blow_film applied: dia={}mm layflat={}mm BUR={}",
                  in.bubble_dia_mm, in.layflat_width_mm, in.bur);

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

}  // namespace koocadcam::skill::blow_film
