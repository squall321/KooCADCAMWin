// @lat: [[engine/skills#hazmat_classify]]

#include "hazmat_classify.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cctype>

namespace koocadcam::skill::hazmat_classify {

using nlohmann::json;

namespace {

bool isValidUnNumber(const std::string& s)
{
    // Format: "UN" + 4 digits, e.g. "UN1090".
    if (s.size() != 6) return false;
    if (s[0] != 'U' || s[1] != 'N') return false;
    for (std::size_t i = 2; i < 6; ++i) {
        if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
    }
    return true;
}

bool isValidClass(const std::string& s)
{
    return s == "1" || s == "2" || s == "3" || s == "4" || s == "5" ||
           s == "6" || s == "7" || s == "8" || s == "9";
}

bool isValidPackingGroup(const std::string& s)
{
    return s.empty() || s == "I" || s == "II" || s == "III";
}

bool classHasNoPackingGroup(const std::string& s)
{
    // Per DOT/ADR — classes 1 (explosives), 2 (gases), 7 (radioactive)
    // do NOT assign a packing group.
    return s == "1" || s == "2" || s == "7";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.un_number.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": un_number must not be empty");
    } else if (!isValidUnNumber(in.un_number)) {
        r.add("DFM-HM-UNNUMBER", "error",
              std::string(kSkillId) + ": un_number '" + in.un_number +
              "' invalid — expected 'UN' + 4 digits");
    }

    if (in.hazard_class.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": hazard_class must not be empty");
    } else if (!isValidClass(in.hazard_class)) {
        r.add("DFM-HM-CLASS", "error",
              std::string(kSkillId) + ": hazard_class '" + in.hazard_class +
              "' invalid — expected '1' through '9'");
    }

    if (!isValidPackingGroup(in.packing_group)) {
        r.add("DFM-HM-PKG", "error",
              std::string(kSkillId) + ": packing_group '" + in.packing_group +
              "' invalid — expected I | II | III | ''");
    }

    if (isValidClass(in.hazard_class)
        && classHasNoPackingGroup(in.hazard_class)
        && !in.packing_group.empty()) {
        r.add("DFM-HM-PKG-MISMATCH", "info",
              std::string(kSkillId) + ": hazard_class " + in.hazard_class +
              " does not assign a packing group — given '" +
              in.packing_group + "'");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hazmat_classify DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string pkg_effective =
        classHasNoPackingGroup(in.hazard_class) ? "" : in.packing_group;

    json params = {
        { "un_number",     in.un_number },
        { "hazard_class",  in.hazard_class },
        { "packing_group", in.packing_group },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_inspection_only",   true },
        { "geometric_change",     false },
        { "un_number",            in.un_number },
        { "hazard_class",         in.hazard_class },
        { "packing_group",        in.packing_group },
        { "packing_group_effective", pkg_effective },
        { "is_radioactive",       in.hazard_class == "7" },
        { "is_explosive",         in.hazard_class == "1" },
        { "is_gas",               in.hazard_class == "2" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "hazmat_record";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Hazmat shipper declaration ~ 20 minutes.
    tooling.est_cycle_time_s  = 1200.0;
    tooling.extra["inspection_type"] = "hazmat_classify";
    tooling.extra["un_number"]       = in.un_number;
    tooling.extra["hazard_class"]    = in.hazard_class;
    tooling.extra["packing_group"]   = in.packing_group;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::hazmat_classify applied: un={} class={} pkg={}",
        in.un_number, in.hazard_class, in.packing_group);

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

}  // namespace koocadcam::skill::hazmat_classify
