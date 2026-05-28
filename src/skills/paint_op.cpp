// @lat: [[engine/skills#paint_op]]

#include "paint_op.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::paint_op {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "paint_op thickness_um must be > 0");
    } else {
        if (in.thickness_um < 10.0) {
            r.add("DFM-PAINT-THK", "error",
                  "paint_op thickness_um " + std::to_string(in.thickness_um) +
                  " < 10 μm — below practical wet-film coverage");
        }
        if (in.thickness_um > 500.0) {
            r.add("DFM-PAINT-THK", "error",
                  "paint_op thickness_um " + std::to_string(in.thickness_um) +
                  " > 500 μm — exceeds single-pass sag-resistance limit "
                  "(specify multiple passes)");
        }
    }

    if (in.paint_type == "powder") {
        r.add("DFM-PAINT-TYPE", "warning",
              "paint_op paint_type='powder' — use the powder_coat skill "
              "instead (electrostatic application + heat cure)");
    } else if (in.paint_type != "wet") {
        r.add("DFM-PAINT-TYPE", "info",
              "paint_op paint_type '" + in.paint_type +
              "' unrecognised — expected 'wet' (or 'powder' → powder_coat)");
    }

    if (in.cure_temp_c < 10.0 || in.cure_temp_c > 200.0) {
        r.add("DFM-PAINT-CURE", "info",
              "paint_op cure_temp_c " + std::to_string(in.cure_temp_c) +
              " outside typical [10, 200] °C range — verify paint datasheet");
    }
    if (in.cure_time_min < 1.0) {
        r.add("DFM-PAINT-CURE", "info",
              "paint_op cure_time_min " + std::to_string(in.cure_time_min) +
              " < 1 min — likely insufficient cure");
    }
    if (in.color.empty()) {
        r.add("DFM-INPUT", "warning",
              "paint_op color empty — defaulting to black");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "paint_op DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("paint_op: entry_face datum unresolved");

    const std::string color = in.color.empty() ? "black" : in.color;
    const std::string ptype = in.paint_type.empty() ? "wet" : in.paint_type;

    json params = {
        { "entry_face_id", *entryId },
        { "paint_type",    ptype },
        { "color",         color },
        { "thickness_um",  in.thickness_um },
        { "cure_temp_c",   in.cure_temp_c },
        { "cure_time_min", in.cure_time_min },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "target_face_id",   *entryId },
        { "paint_type",       ptype },
        { "color",            color },
        { "thickness_um",     in.thickness_um },
        { "cure_temp_c",      in.cure_temp_c },
        { "cure_time_min",    in.cure_time_min },
        { "cure_schedule_ok",
          in.cure_temp_c >= 10.0 && in.cure_temp_c <= 200.0
              && in.cure_time_min >= 1.0 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "spray_gun";
    tooling.tool_dia_mm       = 1.4;    // typical HVLP nozzle tip
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_nozzle_polymer_paint";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Time ≈ surface area / spray rate (~ 200 mm²/s) + cure time.
    // We can't see surface area until stamped — use a fixed 30 s spray base.
    tooling.est_cycle_time_s  = 30.0 + in.cure_time_min * 60.0;
    tooling.extra = {
        { "process",        "wet_paint_spray" },
        { "paint_type",     ptype },
        { "color",          color },
        { "thickness_um",   in.thickness_um },
        { "cure_temp_c",    in.cure_temp_c },
        { "cure_time_min",  in.cure_time_min },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug(
        "skill::paint_op applied: face={} type={} color={} thk={}μm cure={}C/{}min",
        *entryId, ptype, color, in.thickness_um, in.cure_temp_c, in.cure_time_min);

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::paint_op
