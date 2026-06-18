// @lat: [[engine/skills#linear_hole_array]]

#include "linear_hole_array.hpp"

#include "Workpiece.hpp"
#include "drill_hole.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::linear_hole_array {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& /*wp*/, const Input& in)
{
    DFMReport r;
    if (in.hole_count < 3)
        r.add("DFM-INPUT", "error",
              "linear_hole_array needs >= 3 holes (got " +
              std::to_string(in.hole_count) + ")");
    if (in.hole_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", "linear_hole_array hole_dia_mm must be > 0");
    if (in.pitch_mm <= 0.0)
        r.add("DFM-INPUT", "error", "linear_hole_array pitch_mm must be > 0");
    if (std::hypot(in.dir_x, in.dir_y) < 1e-9)
        r.add("DFM-INPUT", "error", "linear_hole_array direction must be non-zero");
    if (std::abs(in.axis_dir.Z()) < 0.99)
        r.add("DFM-INPUT", "error",
              "linear_hole_array supports a vertical (±Z) hole axis only");
    // Hole-to-hole edge clearance (mirrors DFM-003's 1.5 mm structural minimum).
    if (in.pitch_mm > 0.0) {
        const double gap = in.pitch_mm - in.hole_dia_mm;
        if (gap < 1.5)
            r.add("DFM-003", "warning",
                  "linear array hole-to-hole gap " + std::to_string(gap) +
                  " mm < 1.5 mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    const DFMReport r = validate(wp, in);
    if (!r.passed)
        throw SkillError("linear_hole_array::apply — invalid input (DFM error)");

    const double len = std::hypot(in.dir_x, in.dir_y);
    const double ux  = in.dir_x / len, uy = in.dir_y / len;

    std::shared_ptr<Workpiece> current;
    const Workpiece* src = &wp;
    for (int i = 0; i < in.hole_count; ++i) {
        drill_hole::Input din;
        din.entry_face    = in.entry_face;
        din.position_x_mm = in.start_x_mm + i * in.pitch_mm * ux;
        din.position_y_mm = in.start_y_mm + i * in.pitch_mm * uy;
        din.axis_dir      = in.axis_dir;
        din.diameter_mm   = in.hole_dia_mm;
        din.depth_mm      = in.depth_mm;
        din.through_hole  = in.through_hole;
        current = drill_hole::apply(*src, din).workpiece;
        src = current.get();
    }

    json params = {
        { "hole_count",   in.hole_count },
        { "hole_dia_mm",  in.hole_dia_mm },
        { "start_x_mm",   in.start_x_mm },
        { "start_y_mm",   in.start_y_mm },
        { "direction",    { ux, uy, 0.0 } },
        { "pitch_mm",     in.pitch_mm },
        { "axis_dir",     { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "through_hole", in.through_hole },
        { "depth_mm",     in.depth_mm },
    };
    json pattern = {
        { "kind",        kSkillId },
        { "is_compound", true },
        { "hole_count",  in.hole_count },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill";
    tooling.tool_dia_mm      = in.hole_dia_mm;
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = in.hole_count * M_PI *
                                (in.hole_dia_mm / 2.0) * (in.hole_dia_mm / 2.0) *
                                std::max(in.depth_mm, 1.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    current->addFeature(sig);

    spdlog::debug("skill::linear_hole_array applied: {} holes dia={} pitch={}",
                  in.hole_count, in.hole_dia_mm, in.pitch_mm);

    return SkillOutput{ current, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& /*wp*/)
{
    return {};   // see re::recognizeCompounds
}

}  // namespace koocadcam::skill::linear_hole_array
