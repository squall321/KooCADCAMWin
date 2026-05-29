// @lat: [[engine/skills#glue_laminate]]

#include "glue_laminate.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::glue_laminate {

using nlohmann::json;

namespace {

bool isKnownAdhesive(const std::string& a)
{
    return a == "MUF" || a == "PRF" || a == "PUR";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.ply_count < 2) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": ply_count must be ≥ 2 (got " +
              std::to_string(in.ply_count) + ") — glulam needs ≥ 2 plies");
    }

    if (in.ply_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": ply_thickness_mm must be > 0");
    } else {
        if (in.ply_thickness_mm < 10.0) {
            r.add("DFM-GLULAM-PLY", "error",
                  std::string(kSkillId) + ": ply_thickness_mm " +
                  std::to_string(in.ply_thickness_mm) +
                  " < 10 mm — too thin for structural lamination");
        }
        if (in.ply_thickness_mm > 50.0) {
            r.add("DFM-GLULAM-PLY", "error",
                  std::string(kSkillId) + ": ply_thickness_mm " +
                  std::to_string(in.ply_thickness_mm) +
                  " > 50 mm — exceeds EN 14080 §5.1.4 max lamination thickness");
        }
    }

    if (!isKnownAdhesive(in.adhesive_type)) {
        r.add("DFM-GLULAM-ADHESIVE", "error",
              std::string(kSkillId) + ": adhesive_type '" + in.adhesive_type +
              "' not in {MUF, PRF, PUR}");
    }

    if (in.ply_count >= 2 && in.ply_thickness_mm > 0.0) {
        const double total = in.ply_count * in.ply_thickness_mm;
        if (total > 1800.0) {
            r.add("DFM-GLULAM-DEPTH", "info",
                  std::string(kSkillId) + ": total_thickness " +
                  std::to_string(total) +
                  " mm > 1800 mm — verify transport / lift envelope");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "glue_laminate DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double total_thickness_mm =
        in.ply_count * in.ply_thickness_mm;

    json params = {
        { "ply_count",        in.ply_count },
        { "ply_thickness_mm", in.ply_thickness_mm },
        { "adhesive_type",    in.adhesive_type },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "ply_count",           in.ply_count },
        { "ply_thickness_mm",    in.ply_thickness_mm },
        { "adhesive_type",       in.adhesive_type },
        { "total_thickness_mm",  total_thickness_mm },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "hydraulic_press";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cure times: MUF ~ 8 h, PRF ~ 6 h, PUR ~ 4 h.  Lay-up adds ~ 5 min/ply.
    double cure_h = 8.0;
    if (in.adhesive_type == "PRF") cure_h = 6.0;
    else if (in.adhesive_type == "PUR") cure_h = 4.0;
    tooling.est_cycle_time_s  = cure_h * 3600.0 +
                                300.0 * static_cast<double>(in.ply_count);
    tooling.extra = {
        { "process",             "glulam_lay_up_press_cure" },
        { "ply_count",           in.ply_count },
        { "ply_thickness_mm",    in.ply_thickness_mm },
        { "adhesive_type",       in.adhesive_type },
        { "total_thickness_mm",  total_thickness_mm },
        { "code_reference",      "EN_14080" },
        { "process_category",    "construction_glulam" },
        { "geometric_no_op",     true },
        { "dfm_findings",        findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::glue_laminate applied: {} plies × {}mm ({}mm total) adhesive={}",
        in.ply_count, in.ply_thickness_mm, total_thickness_mm, in.adhesive_type);

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

}  // namespace koocadcam::skill::glue_laminate
