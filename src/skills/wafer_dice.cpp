// @lat: [[engine/skills#wafer_dice]]

#include "wafer_dice.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>

namespace koocadcam::skill::wafer_dice {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wafer_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": wafer_dia_mm must be > 0 (got " +
              std::to_string(in.wafer_dia_mm) + ")");
    } else if (in.wafer_dia_mm > 450.0) {
        r.add("DFM-WAFER-DIA", "info",
              std::string(kSkillId) + ": wafer_dia_mm " +
              std::to_string(in.wafer_dia_mm) +
              " exceeds 450 mm — largest production diameter is 450 mm");
    }

    if (in.die_size_x_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": die_size_x_mm must be > 0 (got " +
              std::to_string(in.die_size_x_mm) + ")");
    } else if (in.die_size_x_mm >= in.wafer_dia_mm) {
        r.add("DFM-DIE-X", "error",
              std::string(kSkillId) + ": die_size_x_mm " +
              std::to_string(in.die_size_x_mm) +
              " must be < wafer_dia_mm");
    }

    if (in.die_size_y_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": die_size_y_mm must be > 0 (got " +
              std::to_string(in.die_size_y_mm) + ")");
    } else if (in.die_size_y_mm >= in.wafer_dia_mm) {
        r.add("DFM-DIE-Y", "error",
              std::string(kSkillId) + ": die_size_y_mm " +
              std::to_string(in.die_size_y_mm) +
              " must be < wafer_dia_mm");
    }

    if (in.kerf_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": kerf_um must be > 0 (got " +
              std::to_string(in.kerf_um) + ")");
    } else {
        // kerf must be smaller than the smaller die dimension (in μm)
        const double minDie_um =
            std::min(in.die_size_x_mm, in.die_size_y_mm) * 1000.0;
        if (in.kerf_um >= minDie_um) {
            r.add("DFM-KERF", "error",
                  std::string(kSkillId) + ": kerf_um " +
                  std::to_string(in.kerf_um) +
                  " ≥ min die dimension μm — kerf would consume whole dies");
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
        std::string msg = "wafer_dice DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Usable wafer area ≈ π · (d/2)² minus 5 % edge exclusion.
    const double r_mm    = 0.5 * in.wafer_dia_mm;
    const double area    = M_PI * r_mm * r_mm * 0.95;
    const double pitchX  = in.die_size_x_mm + in.kerf_um * 1.0e-3;
    const double pitchY  = in.die_size_y_mm + in.kerf_um * 1.0e-3;
    const int    dieCount = static_cast<int>(std::floor(area / (pitchX * pitchY)));

    json params = {
        { "wafer_dia_mm",  in.wafer_dia_mm },
        { "die_size_x_mm", in.die_size_x_mm },
        { "die_size_y_mm", in.die_size_y_mm },
        { "kerf_um",       in.kerf_um },
    };
    json pattern = {
        { "kind",          kSkillId },
        { "wafer_dia_mm",  in.wafer_dia_mm },
        { "die_size_x_mm", in.die_size_x_mm },
        { "die_size_y_mm", in.die_size_y_mm },
        { "kerf_um",       in.kerf_um },
        { "die_count_est", dieCount },
        { "is_separation", true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "dicing_saw";
    tooling.tool_dia_mm       = in.kerf_um * 1.0e-3;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "diamond";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;          // negligible at planning level
    // Two passes per street, ~ 10 s per street; streets ≈ wafer_dia / pitch.
    const double streetsX = std::ceil(in.wafer_dia_mm / pitchX);
    const double streetsY = std::ceil(in.wafer_dia_mm / pitchY);
    tooling.est_cycle_time_s  = 10.0 * (streetsX + streetsY);
    tooling.extra = {
        { "process",        "wafer_dicing" },
        { "die_count_est",  dieCount },
        { "dfm_findings",   findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::wafer_dice applied: Ø={}mm dieXY={}x{}mm kerf={}μm n={}",
                  in.wafer_dia_mm, in.die_size_x_mm, in.die_size_y_mm,
                  in.kerf_um, dieCount);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: no geometric trace at OCCT macroscale.

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

}  // namespace koocadcam::skill::wafer_dice
