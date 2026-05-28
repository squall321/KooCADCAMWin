// @lat: [[engine/skills#ct_scan]]

#include "ct_scan.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::ct_scan {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.voxel_size_mm <= 0.0 || in.voxel_size_mm > 1.0) {
        r.add("DFM-INPUT", "error",
              "ct_scan voxel_size_mm must be in (0, 1] mm (got " +
              std::to_string(in.voxel_size_mm) + ")");
    }
    if (in.dose_kvp < 40.0 || in.dose_kvp > 600.0) {
        r.add("DFM-INPUT", "error",
              "ct_scan dose_kvp must be in [40, 600] (got " +
              std::to_string(in.dose_kvp) + ")");
    }
    if (in.defect_size_target_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ct_scan defect_size_target_mm must be > 0 (got " +
              std::to_string(in.defect_size_target_mm) + ")");
    }

    // Nyquist: cannot resolve features smaller than the voxel.
    if (in.voxel_size_mm > 0.0 && in.defect_size_target_mm > 0.0 &&
        in.voxel_size_mm > in.defect_size_target_mm)
    {
        r.add("DFM-CT-NYQUIST", "error",
              "ct_scan voxel_size " + std::to_string(in.voxel_size_mm) +
              " mm > defect_size_target " +
              std::to_string(in.defect_size_target_mm) +
              " mm — cannot resolve target defect (Nyquist)");
    }

    if (in.dose_kvp > 450.0 && in.dose_kvp <= 600.0) {
        r.add("DFM-CT-DOSE", "warning",
              "ct_scan dose_kvp " + std::to_string(in.dose_kvp) +
              " > 450 — high-energy CT, shielding + worker dose review required");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ct_scan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // CT can detect internal defects whenever Nyquist is satisfied.
    const bool detectInternal =
        in.voxel_size_mm <= in.defect_size_target_mm;

    json params = {
        { "voxel_size_mm",          in.voxel_size_mm },
        { "dose_kvp",               in.dose_kvp },
        { "defect_size_target_mm",  in.defect_size_target_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_inspection_only",         true },
        { "geometric_change",           false },
        { "voxel_size_mm",              in.voxel_size_mm },
        { "dose_kvp",                   in.dose_kvp },
        { "defect_size_target_mm",      in.defect_size_target_mm },
        { "detect_internal_defects",    detectInternal },
        { "inspection_method",          "computed_tomography_3d" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "ct_scanner";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tungsten_target";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle: setup (~ 300 s) + projections (~ 600 s at 50 µm) + recon (~ 300 s).
    // Smaller voxels → many more projections, so scale inversely.
    const double base = 600.0;
    const double scale = (in.voxel_size_mm > 0.0) ? (0.050 / in.voxel_size_mm) : 1.0;
    tooling.est_cycle_time_s  = 300.0 + base * std::max(1.0, scale) + 300.0;
    tooling.extra["dfm_findings"]              = findings;
    tooling.extra["detect_internal_defects"]   = detectInternal;
    tooling.extra["machining_constraint"]      =
        "inspection — X-ray CT, no material removal";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ct_scan applied: voxel={} mm dose={} kVp target_defect={} mm",
                  in.voxel_size_mm, in.dose_kvp, in.defect_size_target_mm);

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

}  // namespace koocadcam::skill::ct_scan
