// @lat: [[engine/skills#bolt_circle_pattern]]

#include "bolt_circle_pattern.hpp"

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

namespace koocadcam::skill::bolt_circle_pattern {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& /*wp*/, const Input& in)
{
    DFMReport r;
    if (in.hole_count < 3)
        r.add("DFM-INPUT", "error",
              "bolt_circle_pattern needs >= 3 holes (got " +
              std::to_string(in.hole_count) + ")");
    if (in.hole_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", "bolt_circle hole_dia_mm must be > 0");
    if (in.bolt_circle_dia_mm <= in.hole_dia_mm)
        r.add("DFM-INPUT", "error",
              "bolt_circle_dia_mm must exceed hole_dia_mm");

    // Hole-to-hole edge clearance along the pitch circle (mirrors DFM-003's
    // 1.5 mm structural minimum): chord between adjacent holes minus both radii.
    if (in.hole_count >= 3 && in.bolt_circle_dia_mm > 0.0) {
        const double R     = in.bolt_circle_dia_mm / 2.0;
        const double chord = 2.0 * R * std::sin(M_PI / in.hole_count);
        const double gap   = chord - in.hole_dia_mm;
        if (gap < 1.5)
            r.add("DFM-003", "warning",
                  "bolt-circle hole-to-hole gap " + std::to_string(gap) +
                  " mm < 1.5 mm (holes too dense for the pitch circle)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    const DFMReport r = validate(wp, in);
    if (!r.passed)
        throw SkillError("bolt_circle_pattern::apply — invalid input (DFM error)");

    const double R = in.bolt_circle_dia_mm / 2.0;

    // Compose drill_hole::apply per hole so entry/through resolution and the
    // chain contract are inherited from the proven atom.  Each call carries the
    // running workpiece (and its feature history) into the next.
    std::shared_ptr<Workpiece> current;
    const Workpiece* src = &wp;
    for (int i = 0; i < in.hole_count; ++i) {
        const double ang =
            (in.start_angle_deg + i * 360.0 / in.hole_count) * M_PI / 180.0;
        drill_hole::Input din;
        din.entry_face    = in.entry_face;
        din.position_x_mm = in.center_x_mm + R * std::cos(ang);
        din.position_y_mm = in.center_y_mm + R * std::sin(ang);
        din.axis_dir      = in.axis_dir;
        din.diameter_mm   = in.hole_dia_mm;
        din.depth_mm      = in.depth_mm;
        din.through_hole  = in.through_hole;
        current = drill_hole::apply(*src, din).workpiece;
        src = current.get();
    }

    // Stamp the compound signature on top of the per-hole drill signatures.
    json params = {
        { "hole_count",         in.hole_count },
        { "bolt_circle_dia_mm", in.bolt_circle_dia_mm },
        { "hole_dia_mm",        in.hole_dia_mm },
        { "center_x_mm",        in.center_x_mm },
        { "center_y_mm",        in.center_y_mm },
        { "axis_dir",           { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "through_hole",       in.through_hole },
        { "depth_mm",           in.depth_mm },
        { "start_angle_deg",    in.start_angle_deg },
    };
    json pattern = {
        { "kind",       kSkillId },
        { "is_compound", true },
        { "hole_count", in.hole_count },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = in.hole_dia_mm;
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm  = 300.0;
    tooling.feed_per_tooth_mm  = 0.05;
    tooling.stock_removed_mm3  = in.hole_count * M_PI *
                                 (in.hole_dia_mm / 2.0) * (in.hole_dia_mm / 2.0) *
                                 std::max(in.depth_mm, 1.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    current->addFeature(sig);

    spdlog::debug("skill::bolt_circle_pattern applied: {} holes dia={} on PCD={}",
                  in.hole_count, in.hole_dia_mm, in.bolt_circle_dia_mm);

    return SkillOutput{ current, sig };
}

// ── Recognition (handled by the koo_re grammar) ──────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& /*wp*/)
{
    return {};   // see re::recognizeCompounds
}

}  // namespace koocadcam::skill::bolt_circle_pattern
