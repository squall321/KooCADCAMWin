// @lat: [[engine/skills#tube_to_tubesheet_weld]]

#include "tube_to_tubesheet_weld.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::tube_to_tubesheet_weld {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tube_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": tube_dia_mm must be > 0 (got " +
              std::to_string(in.tube_dia_mm) + ")");
    }
    if (in.sheet_thick_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sheet_thick_mm must be > 0 (got " +
              std::to_string(in.sheet_thick_mm) + ")");
    }
    if (in.joint_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": joint_count must be > 0 (got " +
              std::to_string(in.joint_count) + ")");
    }

    if (in.tube_dia_mm > 0.0 && in.tube_dia_mm < 6.0) {
        r.add("DFM-HE-TUBE-DIA", "info",
              std::string(kSkillId) + ": tube_dia_mm " +
              std::to_string(in.tube_dia_mm) +
              " < 6 mm — below typical shell-and-tube range, capillary "
              "regime; manual TIG may struggle to fill annulus");
    }
    if (in.tube_dia_mm > 50.0) {
        r.add("DFM-HE-TUBE-DIA", "info",
              std::string(kSkillId) + ": tube_dia_mm " +
              std::to_string(in.tube_dia_mm) +
              " > 50 mm — pipe-class diameter, prefer butt-weld + reinforced "
              "tubesheet hole");
    }

    if (in.tube_dia_mm > 0.0 && in.sheet_thick_mm > 0.0 &&
        in.sheet_thick_mm < in.tube_dia_mm * 0.2) {
        r.add("DFM-HE-SHEET-THK", "info",
              std::string(kSkillId) + ": sheet_thick_mm " +
              std::to_string(in.sheet_thick_mm) +
              " < 0.2 × tube_dia_mm — joint may not develop full strength");
    }

    if (in.joint_count > 10000) {
        r.add("DFM-HE-COUNT", "info",
              std::string(kSkillId) + ": joint_count " +
              std::to_string(in.joint_count) +
              " > 10000 — batch / fixture / heat-input planning concern");
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

    // Fillet leg ≈ tube wall thickness; for slice-1 we approximate as
    // 12 % of the tube OD (an empirical mean for HX tubing).
    const double filletLegMm = in.tube_dia_mm * 0.12;

    json params = {
        { "tube_dia_mm",    in.tube_dia_mm },
        { "sheet_thick_mm", in.sheet_thick_mm },
        { "joint_count",    in.joint_count },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "tube_dia_mm",       in.tube_dia_mm },
        { "sheet_thick_mm",    in.sheet_thick_mm },
        { "joint_count",       in.joint_count },
        { "weld_process",      "tig_roll" },
        { "fillet_leg_mm",     filletLegMm },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({ { "code", f.code },
                             { "severity", f.severity },
                             { "message", f.message } });

    ToolingMeta tooling;
    tooling.tool_type         = "tig_torch_orbital";
    tooling.tool_dia_mm       = in.tube_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tungsten";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Orbital TIG ≈ 60 s / joint; scale with count.
    tooling.est_cycle_time_s  = 60.0 * static_cast<double>(in.joint_count);
    tooling.extra = {
        { "process",        "tig_roll_and_weld" },
        { "joint_count",    in.joint_count },
        { "fillet_leg_mm",  filletLegMm },
        { "dfm_findings",   findings },
        { "process_category", "heat_exchanger_fab" },
        { "geometric_no_op",  true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tube_to_tubesheet_weld applied: dia={} sheet={} n={}",
                  in.tube_dia_mm, in.sheet_thick_mm, in.joint_count);

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

}  // namespace koocadcam::skill::tube_to_tubesheet_weld
