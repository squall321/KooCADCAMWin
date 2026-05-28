// @lat: [[engine/skills#palletize]]

#include "palletize.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::palletize {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.rows < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": rows must be ≥ 1 (got " +
              std::to_string(in.rows) + ")");
    }
    if (in.cols < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cols must be ≥ 1 (got " +
              std::to_string(in.cols) + ")");
    }
    if (in.layers < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": layers must be ≥ 1 (got " +
              std::to_string(in.layers) + ")");
    }

    if (in.pallet_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pallet_length_mm must be > 0");
    }
    if (in.pallet_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pallet_width_mm must be > 0");
    }

    if (in.rows >= 1 && in.cols >= 1 && in.layers >= 1) {
        const int total = in.rows * in.cols * in.layers;
        if (total > 5000) {
            r.add("DFM-PALLET-COUNT", "info",
                  std::string(kSkillId) + ": parts_per_pallet " +
                  std::to_string(total) +
                  " > 5000 — verify stack stability");
        }
    }

    // ISO standard pallet 1200 × 1000 (and EUR 1200 × 800).  Anything
    // beyond is non-standard.
    if (in.pallet_length_mm > 1200.0 || in.pallet_width_mm > 1000.0) {
        r.add("DFM-PALLET-SIZE", "info",
              std::string(kSkillId) + ": pallet_size " +
              std::to_string(in.pallet_length_mm) + " × " +
              std::to_string(in.pallet_width_mm) +
              " exceeds ISO 1200×1000 envelope");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "palletize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int parts_per_pallet = in.rows * in.cols * in.layers;

    json pallet_pattern_obj = {
        { "rows",   in.rows },
        { "cols",   in.cols },
        { "layers", in.layers },
    };
    json pallet_size = { in.pallet_length_mm, in.pallet_width_mm };

    json params = {
        { "rows",             in.rows },
        { "cols",             in.cols },
        { "layers",           in.layers },
        { "pallet_length_mm", in.pallet_length_mm },
        { "pallet_width_mm",  in.pallet_width_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "pallet_pattern",    pallet_pattern_obj },
        { "parts_per_pallet",  parts_per_pallet },
        { "pallet_size",       pallet_size },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "palletizer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Typical robotic palletizer: ~3 s per pick+place.  Roll up to full pallet.
    tooling.est_cycle_time_s  = 3.0 * static_cast<double>(parts_per_pallet);
    tooling.extra = {
        { "process",           "robotic_palletize" },
        { "pallet_pattern",    pallet_pattern_obj },
        { "parts_per_pallet",  parts_per_pallet },
        { "pallet_size",       pallet_size },
        { "process_category",  "material_handling" },
        { "geometric_no_op",   true },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::palletize applied: pattern={}x{}x{} ({} parts) on {}x{}mm pallet",
        in.rows, in.cols, in.layers, parts_per_pallet,
        in.pallet_length_mm, in.pallet_width_mm);

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

}  // namespace koocadcam::skill::palletize
