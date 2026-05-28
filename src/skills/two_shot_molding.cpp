// @lat: [[engine/skills#two_shot_molding]]

#include "two_shot_molding.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>
#include <utility>

namespace koocadcam::skill::two_shot_molding {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownPartings()
{
    static const std::unordered_set<std::string> s = {
        "planar", "stepped", "complex",
    };
    return s;
}

// Material-pair bonding compatibility (info-only).  Anti-patterns return a
// short reason; everything else is presumed OK.
const char* bondAdvisoryFor(const std::string& a, const std::string& b)
{
    // Olefins (PE / PP) bond poorly to almost everything without primer.
    auto isOlefin = [](const std::string& m) {
        return m == "pp" || m == "polypropylene" ||
               m == "pe" || m == "polyethylene";
    };
    if (isOlefin(a) && !isOlefin(b))
        return "polyolefin substrate requires primer for non-olefin overmold";
    if (isOlefin(b) && !isOlefin(a))
        return "polyolefin overmold requires primer over non-olefin substrate";
    // ABS / PC / nylon all bond well to TPE-S, TPE-V.  Silicone needs primer
    // on almost everything.
    if (a == "silicone" || b == "silicone")
        return "silicone over thermoplastic requires plasma/primer treatment";
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.material_1.empty()) {
        r.add("DFM-2SHOT-MATERIAL", "error",
              "two_shot_molding: material_1 must be specified");
    }
    if (in.material_2.empty()) {
        r.add("DFM-2SHOT-MATERIAL", "error",
              "two_shot_molding: material_2 must be specified");
    }
    if (!in.material_1.empty() && in.material_1 == in.material_2) {
        r.add("DFM-2SHOT-MATERIAL", "info",
              "two_shot_molding: material_1 and material_2 identical — "
              "consider a single-shot mold");
    }

    if (knownPartings().count(in.parting_geometry) == 0) {
        r.add("DFM-2SHOT-PARTING", "info",
              "two_shot_molding: parting_geometry '" + in.parting_geometry +
              "' unrecognised — expected 'planar' | 'stepped' | 'complex'");
    }

    if (!in.material_1.empty() && !in.material_2.empty()) {
        if (const char* msg = bondAdvisoryFor(in.material_1, in.material_2)) {
            r.add("DFM-2SHOT-COMPAT", "info",
                  std::string("two_shot_molding: ") + msg);
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
        std::string msg = "two_shot_molding DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "material_1",        in.material_1 },
        { "material_2",        in.material_2 },
        { "parting_geometry",  in.parting_geometry },
    };

    json pattern = {
        { "kind",                kSkillId },
        { "material_1",          in.material_1 },
        { "material_2",          in.material_2 },
        { "parting_geometry",    in.parting_geometry },
        { "geometric_change",    false },
        { "process_family",      "injection_molding" },
        { "is_bicomponent",      true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "two_shot_mold";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "P20_tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;       // set by mold-flow planner
    tooling.extra = {
        { "process",            "two_shot_molding" },
        { "process_family",     "injection_molding" },
        { "material_1",         in.material_1 },
        { "material_2",         in.material_2 },
        { "parting_geometry",   in.parting_geometry },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, std::move(params), std::move(pattern),
                          std::move(tooling) };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::two_shot_molding applied: m1={} m2={} parting={}",
                  in.material_1, in.material_2, in.parting_geometry);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only.  A two-shot part's analytic B-Rep is indistinguishable
// from a single-shot part of the same shape.

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

}  // namespace koocadcam::skill::two_shot_molding
