// @lat: [[engine/skills#stone_cut]]

#include "stone_cut.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::stone_cut {

using nlohmann::json;

namespace {

bool isKnownBlade(const std::string& b)
{
    return b == "diamond_circular" || b == "diamond_wire" ||
           b == "abrasive_disc";
}

bool isKnownStone(const std::string& s)
{
    return s == "granite" || s == "marble" || s == "limestone" ||
           s == "sandstone" || s == "basalt";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cut_length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cut_length_m must be > 0 (got " +
              std::to_string(in.cut_length_m) + ")");
    }

    if (!isKnownStone(in.stone_type)) {
        r.add("DFM-INPUT", "warning",
              std::string(kSkillId) + ": stone_type '" + in.stone_type +
              "' not in known set {granite, marble, limestone, sandstone, basalt}");
    }

    if (!isKnownBlade(in.blade_type)) {
        r.add("DFM-STONE-BLADE", "info",
              std::string(kSkillId) + ": blade_type '" + in.blade_type +
              "' not in {diamond_circular, diamond_wire, abrasive_disc}");
    }

    if (in.blade_type == "abrasive_disc" &&
        (in.stone_type == "granite" || in.stone_type == "basalt")) {
        r.add("DFM-STONE-BLADE-FIT", "warning",
              std::string(kSkillId) + ": abrasive_disc on hard stone '" +
              in.stone_type + "' wears rapidly — prefer diamond_circular");
    }

    if (in.cut_length_m > 6.0) {
        r.add("DFM-STONE-LENGTH", "info",
              std::string(kSkillId) + ": cut_length_m " +
              std::to_string(in.cut_length_m) +
              " > 6 m — requires gantry bridge saw");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "stone_cut DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "stone_type",   in.stone_type },
        { "cut_length_m", in.cut_length_m },
        { "blade_type",   in.blade_type },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "stone_type",          in.stone_type },
        { "cut_length_m",        in.cut_length_m },
        { "blade_type",          in.blade_type },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type =
        (in.blade_type == "diamond_wire")     ? "diamond_wire_saw"
      : (in.blade_type == "abrasive_disc")    ? "abrasive_cutoff"
                                              : "diamond_bridge_saw";
    tooling.tool_dia_mm       =
        (in.blade_type == "diamond_circular") ? 600.0 : 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     =
        (in.blade_type == "abrasive_disc") ? "alumina_resin" : "diamond_segment";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Diamond bridge saw on granite: ~0.5 m/min ⇒ 120 s/m.  Wire: ~0.3 m/min.
    {
        const double secPerM =
            (in.blade_type == "diamond_wire") ? 200.0
          : (in.blade_type == "abrasive_disc") ? 240.0
                                               : 120.0;
        tooling.est_cycle_time_s = in.cut_length_m * secPerM;
    }
    tooling.extra = {
        { "process",      "dimension_stone_cutting" },
        { "stone_type",   in.stone_type },
        { "cut_length_m", in.cut_length_m },
        { "blade_type",   in.blade_type },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::stone_cut applied: stone={} length={}m blade={}",
                  in.stone_type, in.cut_length_m, in.blade_type);

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

}  // namespace koocadcam::skill::stone_cut
