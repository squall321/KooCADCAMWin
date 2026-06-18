// @lat: [[engine/skills#coaxial_step_bore]]

#include "coaxial_step_bore.hpp"

#include "Workpiece.hpp"
#include "drill_hole.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::coaxial_step_bore {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& /*wp*/, const Input& in)
{
    DFMReport r;
    if (in.steps.size() < 3)
        r.add("DFM-INPUT", "error",
              "coaxial_step_bore needs >= 3 steps (got " +
              std::to_string(in.steps.size()) + ")");
    for (const auto& s : in.steps) {
        if (s.diameter_mm <= 0.0)
            r.add("DFM-INPUT", "error", "coaxial_step_bore step diameter must be > 0");
        if (!s.through && s.depth_mm <= 0.0)
            r.add("DFM-INPUT", "error", "coaxial_step_bore blind step depth must be > 0");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    const DFMReport r = validate(wp, in);
    if (!r.passed)
        throw SkillError("coaxial_step_bore::apply — invalid input (DFM error)");

    // Bore each step's diameter from the entry face down to its cumulative
    // depth; the widest/shallowest first or last is immaterial (all cuts are
    // coaxial material removal from the same face).
    std::shared_ptr<Workpiece> current;
    const Workpiece* src = &wp;
    double maxDia = 0.0;
    for (const auto& s : in.steps) {
        drill_hole::Input din;
        din.entry_face    = in.entry_face;
        din.position_x_mm = in.center_x_mm;
        din.position_y_mm = in.center_y_mm;
        din.axis_dir      = in.axis_dir;
        din.diameter_mm   = s.diameter_mm;
        din.depth_mm      = s.depth_mm;
        din.through_hole  = s.through;
        current = drill_hole::apply(*src, din).workpiece;
        src = current.get();
        maxDia = std::max(maxDia, s.diameter_mm);
    }

    json stepArr = json::array();
    double minDia = in.steps.empty() ? 0.0 : in.steps.front().diameter_mm;
    for (const auto& s : in.steps) {
        stepArr.push_back({ { "diameter_mm", s.diameter_mm },
                            { "depth_mm", s.depth_mm }, { "through", s.through } });
        minDia = std::min(minDia, s.diameter_mm);
    }
    json params = {
        { "step_count",    static_cast<int>(in.steps.size()) },
        { "steps",         stepArr },
        { "max_dia_mm",    maxDia },
        { "min_dia_mm",    minDia },
        { "position_x_mm", in.center_x_mm },
        { "position_y_mm", in.center_y_mm },
        { "axis_dir",      { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    json pattern = {
        { "kind",        kSkillId },
        { "is_compound", true },
        { "step_count",  static_cast<int>(in.steps.size()) },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "boring_bar";
    tooling.tool_dia_mm      = maxDia;
    tooling.flute_count      = 1;
    tooling.cutting_speed_sfm = 100.0;
    tooling.feed_per_tooth_mm = 0.04;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    current->addFeature(sig);

    spdlog::debug("skill::coaxial_step_bore applied: {} steps max_dia={}",
                  in.steps.size(), maxDia);

    return SkillOutput{ current, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& /*wp*/)
{
    return {};   // see re::recognizeCompounds
}

}  // namespace koocadcam::skill::coaxial_step_bore
