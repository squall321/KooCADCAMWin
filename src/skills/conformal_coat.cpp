// @lat: [[engine/skills#conformal_coat]]

#include "conformal_coat.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>

namespace koocadcam::skill::conformal_coat {

using nlohmann::json;

namespace {

bool isKnownCoatType(const std::string& t)
{
    static const char* kKnown[] = {
        "acrylic",       // AR — UV-cure / solvent, easy rework
        "silicone",      // SR — broad temperature
        "polyurethane",  // UR — abrasion + chemical resistant
        "epoxy",         // ER — rigid, heat resistant
        "parylene",      // XY — CVD, ultra-thin, conformal
    };
    for (const char* k : kKnown) {
        if (t == k) return true;
    }
    return false;
}

const char* ipcClass(double thickness_um)
{
    if (thickness_um < 25.0)  return "IPC-CC-830_thin";
    if (thickness_um > 130.0) return "IPC-CC-830_thick";
    return "IPC-CC-830";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": thickness_um must be > 0 (got " +
              std::to_string(in.thickness_um) + ")");
    } else {
        if (in.thickness_um < 25.0) {
            r.add("DFM-CC-THK-LOW", "error",
                  std::string(kSkillId) + ": thickness_um " +
                  std::to_string(in.thickness_um) +
                  " < 25 μm — below IPC-CC-830 floor");
        }
        if (in.thickness_um > 300.0) {
            r.add("DFM-CC-THK-HIGH", "error",
                  std::string(kSkillId) + ": thickness_um " +
                  std::to_string(in.thickness_um) +
                  " > 300 μm — bubble entrapment + cure-shrink crack risk");
        }
    }

    if (in.coverage_pct <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": coverage_pct must be > 0 (got " +
              std::to_string(in.coverage_pct) + ")");
    } else if (in.coverage_pct > 100.0) {
        r.add("DFM-CC-COVERAGE", "error",
              std::string(kSkillId) + ": coverage_pct " +
              std::to_string(in.coverage_pct) +
              " > 100 — physically impossible");
    } else if (in.coverage_pct < 80.0) {
        r.add("DFM-CC-COVERAGE-LOW", "info",
              std::string(kSkillId) + ": coverage_pct " +
              std::to_string(in.coverage_pct) +
              " < 80 — large keep-out / connector mask zones");
    }

    if (!isKnownCoatType(in.coat_type)) {
        r.add("DFM-CC-TYPE", "info",
              std::string(kSkillId) + ": coat_type '" + in.coat_type +
              "' not in known catalog — treated as custom resin");
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

    json params = {
        { "coat_type",     in.coat_type },
        { "thickness_um",  in.thickness_um },
        { "coverage_pct",  in.coverage_pct },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "coat_type",        in.coat_type },
        { "thickness_um",     in.thickness_um },
        { "coverage_pct",     in.coverage_pct },
        { "ipc_class",        ipcClass(in.thickness_um) },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "conformal_coater";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "spray_or_dip_polymer";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // additive
    // Cycle time ≈ thickness (μm) × 2 s + 60 s flash + cure offline.
    tooling.est_cycle_time_s  = in.thickness_um * 2.0 + 60.0;
    tooling.extra = {
        { "process",          "conformal_coat" },
        { "process_category", "electronics_assembly" },
        { "coat_type",        in.coat_type },
        { "thickness_um",     in.thickness_um },
        { "coverage_pct",     in.coverage_pct },
        { "ipc_class",        ipcClass(in.thickness_um) },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::conformal_coat applied: type={} thk={}μm cov={}%",
                  in.coat_type, in.thickness_um, in.coverage_pct);

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

}  // namespace koocadcam::skill::conformal_coat
