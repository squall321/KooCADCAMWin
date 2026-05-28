// @lat: [[engine/skills#pvd_coat]]

#include "pvd_coat.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::pvd_coat {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pvd_coat thickness_um must be > 0");
    } else {
        if (in.thickness_um < 0.5) {
            r.add("DFM-PVD-THK", "error",
                  "pvd_coat thickness_um " + std::to_string(in.thickness_um) +
                  " < 0.5 μm — below practical PVD nucleation thickness");
        }
        if (in.thickness_um > 10.0) {
            r.add("DFM-PVD-THK", "error",
                  "pvd_coat thickness_um " + std::to_string(in.thickness_um) +
                  " > 10 μm — exceeds typical PVD ceiling "
                  "(internal stress / delamination risk)");
        }
    }

    const cc::PvdCompoundEntry* compound = cc::findPvdCompound(in.coating_compound);
    if (!compound) {
        r.add("DFM-PVD-COMPOUND", "info",
              "pvd_coat coating_compound '" + in.coating_compound +
              "' not in known table {TiN, TiAlN, CrN, DLC} — process plan "
              "will treat as custom chemistry");
    } else if (in.hardness_hv > 0.0) {
        if (in.hardness_hv < compound->hardness_min_hv
            || in.hardness_hv > compound->hardness_max_hv) {
            r.add("DFM-PVD-HARDNESS", "info",
                  "pvd_coat hardness_hv " + std::to_string(in.hardness_hv) +
                  " HV outside expected " + in.coating_compound + " range [" +
                  std::to_string(compound->hardness_min_hv) + ", " +
                  std::to_string(compound->hardness_max_hv) + "] HV — verify"
                  " deposition recipe");
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
        std::string msg = "pvd_coat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("pvd_coat: entry_face datum unresolved");

    const cc::PvdCompoundEntry* compound = cc::findPvdCompound(in.coating_compound);
    const cc::RgbHint rgb = compound ? compound->color_hint
                                     : cc::RgbHint{ 180, 180, 180 };
    double hardness = in.hardness_hv;
    if (hardness <= 0.0 && compound) {
        hardness = 0.5 * (compound->hardness_min_hv + compound->hardness_max_hv);
    }

    json params = {
        { "entry_face_id",    *entryId },
        { "coating_compound", in.coating_compound },
        { "thickness_um",     in.thickness_um },
        { "hardness_hv",      hardness },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "target_face_id",   *entryId },
        { "coating_compound", in.coating_compound },
        { "thickness_um",     in.thickness_um },
        { "hardness_hv",      hardness },
        { "color_rgb_hint",   { rgb.r, rgb.g, rgb.b } },
        { "hardness_in_compound_range",
          compound
            ? (hardness >= compound->hardness_min_hv
               && hardness <= compound->hardness_max_hv)
            : false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "pvd_chamber";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.coating_compound + "_target";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // PVD typical deposition rate ~ 0.5 μm per hour for arc evaporation;
    // include 20 min chamber pump-down floor.
    tooling.est_cycle_time_s  = std::max(20.0 * 60.0,
                                         in.thickness_um / 0.5 * 3600.0);
    tooling.extra = {
        { "process",          "physical_vapor_deposition" },
        { "coating_compound", in.coating_compound },
        { "thickness_um",     in.thickness_um },
        { "hardness_hv",      hardness },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug(
        "skill::pvd_coat applied: face={} compound={} thk={}μm hardness={}HV",
        *entryId, in.coating_compound, in.thickness_um, hardness);

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::pvd_coat
