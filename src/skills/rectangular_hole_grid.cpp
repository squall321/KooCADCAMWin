// @lat: [[engine/skills#rectangular_hole_grid]]

#include "rectangular_hole_grid.hpp"

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

namespace koocadcam::skill::rectangular_hole_grid {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& /*wp*/, const Input& in)
{
    DFMReport r;
    if (in.cols < 2 || in.rows < 2 || in.cols * in.rows < 6)
        r.add("DFM-INPUT", "error",
              "rectangular_hole_grid needs cols,rows >= 2 and >= 6 holes (got " +
              std::to_string(in.cols) + "x" + std::to_string(in.rows) + ")");
    if (in.hole_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", "rectangular_hole_grid hole_dia_mm must be > 0");
    if (in.pitch_u_mm <= 0.0 || in.pitch_v_mm <= 0.0)
        r.add("DFM-INPUT", "error", "rectangular_hole_grid pitches must be > 0");
    if (std::hypot(in.u_dx, in.u_dy) < 1e-9 || std::hypot(in.v_dx, in.v_dy) < 1e-9)
        r.add("DFM-INPUT", "error", "rectangular_hole_grid basis must be non-zero");
    // DFM-003-style minimum web between adjacent holes along each axis.
    const double gapU = in.pitch_u_mm - in.hole_dia_mm;
    const double gapV = in.pitch_v_mm - in.hole_dia_mm;
    if (std::min(gapU, gapV) < 1.5)
        r.add("DFM-003", "warning",
              "grid hole-to-hole gap " + std::to_string(std::min(gapU, gapV)) +
              " mm < 1.5 mm");
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    const DFMReport r = validate(wp, in);
    if (!r.passed)
        throw SkillError("rectangular_hole_grid::apply — invalid input (DFM error)");

    const double ulen = std::hypot(in.u_dx, in.u_dy);
    const double vlen = std::hypot(in.v_dx, in.v_dy);
    const double ux = in.u_dx / ulen, uy = in.u_dy / ulen;
    const double vx = in.v_dx / vlen, vy = in.v_dy / vlen;

    std::shared_ptr<Workpiece> current;
    const Workpiece* src = &wp;
    for (int i = 0; i < in.cols; ++i) {
        for (int j = 0; j < in.rows; ++j) {
            drill_hole::Input din;
            din.entry_face    = in.entry_face;
            din.position_x_mm = in.origin_x_mm + i * in.pitch_u_mm * ux
                                               + j * in.pitch_v_mm * vx;
            din.position_y_mm = in.origin_y_mm + i * in.pitch_u_mm * uy
                                               + j * in.pitch_v_mm * vy;
            din.axis_dir      = in.axis_dir;
            din.diameter_mm   = in.hole_dia_mm;
            din.depth_mm      = in.depth_mm;
            din.through_hole  = in.through_hole;
            current = drill_hole::apply(*src, din).workpiece;
            src = current.get();
        }
    }

    const int holes = in.cols * in.rows;
    json params = {
        { "cols",         in.cols },
        { "rows",         in.rows },
        { "hole_count",   holes },
        { "hole_dia_mm",  in.hole_dia_mm },
        { "origin_x_mm",  in.origin_x_mm },
        { "origin_y_mm",  in.origin_y_mm },
        { "u_dir",        { ux, uy, 0.0 } },
        { "v_dir",        { vx, vy, 0.0 } },
        { "pitch_u_mm",   in.pitch_u_mm },
        { "pitch_v_mm",   in.pitch_v_mm },
        { "axis_dir",     { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "through_hole", in.through_hole },
        { "depth_mm",     in.depth_mm },
    };
    json pattern = {
        { "kind",        kSkillId },
        { "is_compound", true },
        { "hole_count",  holes },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill";
    tooling.tool_dia_mm      = in.hole_dia_mm;
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = holes * M_PI *
                                (in.hole_dia_mm / 2.0) * (in.hole_dia_mm / 2.0) *
                                std::max(in.depth_mm, 1.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    current->addFeature(sig);

    spdlog::debug("skill::rectangular_hole_grid applied: {}x{} holes dia={}",
                  in.cols, in.rows, in.hole_dia_mm);

    return SkillOutput{ current, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& /*wp*/)
{
    return {};   // see re::recognizeCompounds
}

}  // namespace koocadcam::skill::rectangular_hole_grid
