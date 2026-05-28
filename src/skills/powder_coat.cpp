// @lat: [[engine/skills#powder_coat]]

#include "powder_coat.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::powder_coat {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "powder_coat thickness_um must be > 0");
    } else {
        if (in.thickness_um < 50.0) {
            r.add("DFM-PWD-THK", "error",
                  "powder_coat thickness_um " + std::to_string(in.thickness_um) +
                  " < 50 μm — below practical electrostatic coverage");
        }
        if (in.thickness_um > 300.0) {
            r.add("DFM-PWD-THK", "error",
                  "powder_coat thickness_um " + std::to_string(in.thickness_um) +
                  " > 300 μm — above typical single-pass build "
                  "(orange-peel / sag risk)");
        }
    }

    if (in.cure_temp_c < 180.0 || in.cure_temp_c > 220.0) {
        r.add("DFM-PWD-CURE", "info",
              "powder_coat cure_temp_c " + std::to_string(in.cure_temp_c) +
              " outside typical [180, 220] °C range for polyester/epoxy "
              "resins — verify powder datasheet");
    }
    if (in.cure_time_min < 5.0) {
        r.add("DFM-PWD-CURE", "info",
              "powder_coat cure_time_min " + std::to_string(in.cure_time_min) +
              " min < 5 min — likely insufficient cross-link");
    }

    if (!cc::isKnownPowderTexture(in.texture)) {
        r.add("DFM-PWD-TEXTURE", "info",
              "powder_coat texture '" + in.texture +
              "' unrecognised — expected smooth | matte | wrinkle | metallic");
    }

    if (in.color.empty()) {
        r.add("DFM-INPUT", "warning",
              "powder_coat color empty — defaulting to black");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "powder_coat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("powder_coat: entry_face datum unresolved");

    const std::string color = in.color.empty() ? "black" : in.color;
    const std::string tex   = in.texture.empty() ? "smooth" : in.texture;

    json params = {
        { "entry_face_id", *entryId },
        { "color",         color },
        { "thickness_um",  in.thickness_um },
        { "cure_temp_c",   in.cure_temp_c },
        { "cure_time_min", in.cure_time_min },
        { "texture",       tex },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "target_face_id",   *entryId },
        { "color",            color },
        { "thickness_um",     in.thickness_um },
        { "cure_temp_c",      in.cure_temp_c },
        { "cure_time_min",    in.cure_time_min },
        { "texture",          tex },
        { "cure_schedule_ok",
          in.cure_temp_c >= 180.0 && in.cure_temp_c <= 220.0
              && in.cure_time_min >= 5.0 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "electrostatic_powder_gun";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "polymer_powder_resin";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Apply ~ 20 s flat charge + bake cure time.
    tooling.est_cycle_time_s  = 20.0 + in.cure_time_min * 60.0;
    tooling.extra = {
        { "process",         "electrostatic_powder_coat" },
        { "color",           color },
        { "thickness_um",    in.thickness_um },
        { "cure_temp_c",     in.cure_temp_c },
        { "cure_time_min",   in.cure_time_min },
        { "texture",         tex },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug(
        "skill::powder_coat applied: face={} color={} thk={}μm tex={} cure={}C/{}min",
        *entryId, color, in.thickness_um, tex, in.cure_temp_c, in.cure_time_min);

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::powder_coat
