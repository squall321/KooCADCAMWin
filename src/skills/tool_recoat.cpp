// @lat: [[engine/skills#tool_recoat]]

#include "tool_recoat.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <string>

namespace koocadcam::skill::tool_recoat {

using nlohmann::json;

namespace {

constexpr std::array<const char*, 4> kStripMethods = {
    "chemical", "electrochemical", "blast", "thermal",
};

constexpr std::array<const char*, 8> kKnownCoatings = {
    "TiN", "TiCN", "TiAlN", "AlCrN", "AlTiN",
    "DLC", "ZrN", "uncoated",
};

bool isKnownStrip(const std::string& m)
{
    for (const auto* k : kStripMethods) if (m == k) return true;
    return false;
}

bool isKnownCoating(const std::string& c)
{
    for (const auto* k : kKnownCoatings) if (c == k) return true;
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.tool_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": tool_id must not be empty (tool traceability)");
    }
    if (in.new_coating.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": new_coating must not be empty");
    }

    if (!isKnownStrip(in.stripping_method)) {
        r.add("DFM-RC-STRIP", "info",
              std::string(kSkillId) + ": stripping_method '" +
              in.stripping_method +
              "' not in {chemical, electrochemical, blast, thermal}");
    }
    if (!in.new_coating.empty() && !isKnownCoating(in.new_coating)) {
        r.add("DFM-RC-COATING", "info",
              std::string(kSkillId) + ": new_coating '" + in.new_coating +
              "' not in known set");
    }
    if (!in.old_coating.empty() && in.old_coating == in.new_coating) {
        r.add("DFM-RC-SAMECOAT", "info",
              std::string(kSkillId) + ": old_coating == new_coating ('" +
              in.new_coating + "') — verify the strip step is required");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tool_recoat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "tool_id",          in.tool_id },
        { "old_coating",      in.old_coating },
        { "new_coating",      in.new_coating },
        { "stripping_method", in.stripping_method },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "tool_id",           in.tool_id },
        { "old_coating",       in.old_coating },
        { "new_coating",       in.new_coating },
        { "stripping_method",  in.stripping_method },
        { "process",           "strip_and_recoat" },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "coating_chamber";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Strip 30-90 min + PVD/CVD cycle 2-6 hr typical.
    tooling.est_cycle_time_s  = 10800.0;
    tooling.extra["process"]           = "tool_recoat";
    tooling.extra["tool_id"]           = in.tool_id;
    tooling.extra["old_coating"]       = in.old_coating;
    tooling.extra["new_coating"]       = in.new_coating;
    tooling.extra["stripping_method"]  = in.stripping_method;
    tooling.extra["dfm_findings"]      = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tool_recoat applied: tool={} old={} new={} strip={}",
                  in.tool_id, in.old_coating,
                  in.new_coating, in.stripping_method);

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

}  // namespace koocadcam::skill::tool_recoat
