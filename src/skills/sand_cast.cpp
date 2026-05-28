// @lat: [[engine/skills#sand_cast]]

#include "sand_cast.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <unordered_set>

namespace koocadcam::skill::sand_cast {

using nlohmann::json;

namespace {

// Materials commonly produced by sand casting.  Used purely for DFM info
// reporting — unknown materials are NOT errors (process planners may add
// experimental alloys).
const std::unordered_set<std::string>& knownSandCastMaterials()
{
    static const std::unordered_set<std::string> s = {
        "gray_iron", "ductile_iron", "ductile_nodular_iron",
        "aluminum_a356", "aluminum_a319", "aluminum_356", "aluminum_319",
        "carbon_steel", "stainless_steel",
        "bronze",     "brass",
        "copper",     "magnesium",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wall_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sand_cast wall_thickness_mm must be > 0 (got " +
              std::to_string(in.wall_thickness_mm) + ")");
    } else if (in.wall_thickness_mm < 3.0) {
        r.add("DFM-CAST-WALL", "error",
              "sand_cast wall_thickness " +
              std::to_string(in.wall_thickness_mm) +
              " mm < 3.0 mm sand-cast minimum (cannot fill / hot tears)");
    }

    if (in.draft_angle_deg < 1.0) {
        r.add("DFM-CAST-DRAFT", "error",
              "sand_cast draft_angle " +
              std::to_string(in.draft_angle_deg) +
              " deg < 1.0 — pattern cannot release from sand without tearing");
    }

    if (in.material.empty()) {
        r.add("DFM-INPUT", "error", "sand_cast material must be specified");
    } else if (knownSandCastMaterials().count(in.material) == 0) {
        r.add("DFM-CAST-MAT", "info",
              "sand_cast material '" + in.material +
              "' not in standard sand-cast catalog — verify shrink factor");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// NO geometric change — sand_cast is a process that records intent on top
// of an already-existing cavity-inverse shape.  We build a fresh Workpiece
// around the SAME shape so the feature history is appended without mutating
// the source workpiece (consistent with probe_point / scan_pattern).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sand_cast DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "material",            in.material },
        { "wall_thickness_mm",   in.wall_thickness_mm },
        { "draft_angle_deg",     in.draft_angle_deg },
    };
    if (in.parting_line_z_mm.has_value()) {
        params["parting_line_z_mm"] = *in.parting_line_z_mm;
    } else {
        params["parting_line_z_mm"] = nullptr;
    }

    json pattern = {
        { "kind",                   kSkillId },
        { "is_process_only",        true },
        { "geometric_change",       false },
        { "process_family",         "casting" },
        { "process_subtype",        "sand_cast" },
        { "material",               in.material },
        { "wall_thickness_mm",      in.wall_thickness_mm },
        { "draft_angle_deg",        in.draft_angle_deg },
        { "parting_line_z_mm",      in.parting_line_z_mm.has_value()
                                        ? json(*in.parting_line_z_mm)
                                        : json(nullptr) },
        { "additive_workflow",      true },   // pour metal — material added, not removed
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "sand_cast_mold";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "green_sand";   // typical mold media
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;            // no machining
    // Casting cycle time is dominated by pour + cool — a rough hour-scale value.
    tooling.est_cycle_time_s  = 1800.0;
    tooling.extra = {
        { "process",            "sand_cast" },
        { "process_family",     "casting" },
        { "material",           in.material },
        { "wall_thickness_mm",  in.wall_thickness_mm },
        { "draft_angle_deg",    in.draft_angle_deg },
        { "dfm_findings",       findings },
        { "machining_constraint",
          "no material removal — casting process produces final shape directly" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::sand_cast applied: material={} wall={} draft={}",
                  in.material, in.wall_thickness_mm, in.draft_angle_deg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: the shape itself does not carry a discriminator that says
// "I was sand cast" vs "I was investment cast" vs "I was machined from a
// billet."  We therefore only replay history; geometric recognition is not
// attempted.

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

}  // namespace koocadcam::skill::sand_cast
