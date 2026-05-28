// @lat: [[engine/skills#rough_stock_cut]]

#include "rough_stock_cut.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>

namespace koocadcam::skill::rough_stock_cut {

using nlohmann::json;

namespace {

double wastePct(const std::array<double, 3>& orig,
                const std::array<double, 3>& cut)
{
    const double origVol = orig[0] * orig[1] * orig[2];
    const double cutVol  = cut[0]  * cut[1]  * cut[2];
    if (origVol <= 0.0) return 0.0;
    return std::max(0.0, (origVol - cutVol) / origVol * 100.0);
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    for (int i = 0; i < 3; ++i) {
        if (in.original_stock_size[i] <= 0.0) {
            r.add("DFM-INPUT", "error",
                  std::string(kSkillId) + ": original_stock_size[" +
                  std::to_string(i) + "] must be > 0");
        }
        if (in.cut_dimension[i] <= 0.0) {
            r.add("DFM-INPUT", "error",
                  std::string(kSkillId) + ": cut_dimension[" +
                  std::to_string(i) + "] must be > 0");
        }
        if (in.cut_dimension[i] > in.original_stock_size[i] + 1e-6) {
            r.add("DFM-INPUT", "error",
                  std::string(kSkillId) + ": cut_dimension[" +
                  std::to_string(i) + "] " + std::to_string(in.cut_dimension[i]) +
                  " exceeds original_stock_size[" + std::to_string(i) + "] " +
                  std::to_string(in.original_stock_size[i]));
        }
    }
    if (r.passed) {
        const double w = wastePct(in.original_stock_size, in.cut_dimension);
        if (w > 50.0) {
            r.add("RC-WASTE-HIGH", "warning",
                  std::string(kSkillId) + ": waste_pct " +
                  std::to_string(w) + "% > 50% — low material yield");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis (metadata-only) ────────────────────────────────────────────

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

    const double w = wastePct(in.original_stock_size, in.cut_dimension);

    json params = {
        { "original_stock_size",
          { in.original_stock_size[0], in.original_stock_size[1], in.original_stock_size[2] } },
        { "cut_dimension",
          { in.cut_dimension[0], in.cut_dimension[1], in.cut_dimension[2] } },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "original_stock_size",
          { in.original_stock_size[0], in.original_stock_size[1], in.original_stock_size[2] } },
        { "cut_dimension",
          { in.cut_dimension[0], in.cut_dimension[1], in.cut_dimension[2] } },
        { "waste_pct",             w },
        { "geometry_changed",      false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "saw";
    tooling.tool_dia_mm       = 250.0;        // typical horizontal bandsaw blade
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "bi_metal_hss";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Saw-cut cycle time: rough estimate from cut cross-section.
    // Bandsaw across a 100 × 100 stock ≈ 60 s; scale linearly with area.
    const double minDim = std::min({ in.cut_dimension[0],
                                     in.cut_dimension[1],
                                     in.cut_dimension[2] });
    const double midDim = (in.cut_dimension[0] + in.cut_dimension[1]
                          + in.cut_dimension[2]) / 3.0;
    tooling.est_cycle_time_s  = std::max(15.0, minDim * midDim / 167.0);
    tooling.extra["waste_pct"]        = w;
    tooling.extra["geometric_no_op"]  = true;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::rough_stock_cut applied: waste={}%", w);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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

}  // namespace koocadcam::skill::rough_stock_cut
