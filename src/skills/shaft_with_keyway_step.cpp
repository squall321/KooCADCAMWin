// @lat: [[engine/skills#shaft_with_keyway_step]]

#include "shaft_with_keyway_step.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::shaft_with_keyway_step {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.full_dia_mm     <= 0.0 ||
        in.shaft_length_mm <= 0.0 ||
        in.step_dia_mm     <= 0.0 ||
        in.key_w_mm        <= 0.0 ||
        in.key_h_mm        <= 0.0 ||
        in.key_len_mm      <= 0.0) {
        r.add("DFM-INPUT", "error",
              "shaft_with_keyway_step: all geometric inputs must be > 0");
        return r;
    }

    if (in.step_dia_mm >= in.full_dia_mm) {
        r.add("DFM-SHAFT-STEP", "error",
              "shaft_with_keyway_step: step_dia " +
              std::to_string(in.step_dia_mm) +
              " >= full_dia " + std::to_string(in.full_dia_mm) +
              " (no step)");
    }

    if (in.step_z_mm <= 0.0 || in.step_z_mm >= in.shaft_length_mm) {
        r.add("DFM-SHAFT-STEPZ", "error",
              "shaft_with_keyway_step: step_z " +
              std::to_string(in.step_z_mm) +
              " outside (0, shaft_length)");
    }

    // The snap-ring groove typically sits on the small (step) section.
    if (in.snap_ring_z_mm <= in.step_z_mm ||
        in.snap_ring_z_mm >= in.shaft_length_mm) {
        r.add("DFM-SHAFT-SNAPZ", "error",
              "shaft_with_keyway_step: snap_ring_z " +
              std::to_string(in.snap_ring_z_mm) +
              " outside the step section (step_z, shaft_length)");
    }

    // The keyway sits on the LARGE (full_dia) section by convention.
    if (in.key_center_z_mm <= in.key_len_mm / 2.0 ||
        in.key_center_z_mm + in.key_len_mm / 2.0 >= in.step_z_mm) {
        r.add("DFM-INPUT", "error",
              "shaft_with_keyway_step: keyway extends outside the large-Ø section");
    }

    if (in.key_w_mm < 0.8) {
        r.add("DFM-002", "error",
              "shaft_with_keyway_step: key_w < 0.8 mm");
    }
    if (in.key_w_mm >= in.full_dia_mm) {
        r.add("DFM-KEYWAY-FIT", "error",
              "shaft_with_keyway_step: key_w >= full_dia (slot wider than shaft)");
    }

    if (in.snap_ring_depth_mm <= 0.0 ||
        in.snap_ring_depth_mm >= (in.step_dia_mm / 2.0) - 1.0) {
        r.add("DFM-INPUT", "error",
              "shaft_with_keyway_step: snap_ring_depth too deep (or non-positive)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shaft_with_keyway_step DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double fullR    = in.full_dia_mm / 2.0;
    const double stepR    = in.step_dia_mm / 2.0;
    const double Z        = in.shaft_length_mm;
    const double kOverhang = 0.05;

    // ── Tool 1: step turn-down.  Annular ring removal from the +Z portion
    //            above step_z_mm, with inner=stepR, outer=fullR + overhang.
    const double stepLen = Z - in.step_z_mm;
    const gp_Ax2 axStep(gp_Pnt(0, 0, in.step_z_mm), gp::DZ());
    const TopoDS_Shape stepTool =
        pr::annularRing(axStep,
                        fullR + kOverhang,
                        stepR,
                        stepLen + kOverhang);

    // ── Tool 2: sled keyway on the large-Ø section.  Open at the top of
    //            the large section?  Conventional sled keyway is an open
    //            slot of width key_w, depth key_h, length key_len, axis
    //            along Z.  Cut box: width key_w (X), length key_len (Z),
    //            depth key_h going from +Y wall of the shaft inward.
    const double keyZ0 = in.key_center_z_mm - in.key_len_mm / 2.0;
    const gp_Pnt keyOrigin(-in.key_w_mm / 2.0,
                            fullR - in.key_h_mm,
                            keyZ0);
    const gp_Ax2 keyAx(keyOrigin, gp::DZ());
    const TopoDS_Shape keyBox = pr::box(keyAx,
                                        in.key_w_mm,
                                        in.key_h_mm + kOverhang,
                                        in.key_len_mm);

    // ── Tool 3: snap-ring groove.  Annular ring at snap_ring_z, depth
    //            snap_ring_depth_mm, width snap_ring_w_mm.
    const gp_Ax2 axSnap(gp_Pnt(0, 0, in.snap_ring_z_mm - in.snap_ring_w_mm / 2.0),
                        gp::DZ());
    const TopoDS_Shape snapTool =
        pr::annularRing(axSnap,
                        stepR + kOverhang,
                        stepR - in.snap_ring_depth_mm,
                        in.snap_ring_w_mm);

    std::vector<TopoDS_Shape> tools = { stepTool, keyBox, snapTool };
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    const int subCount = 3;

    json params = {
        { "full_dia_mm",        in.full_dia_mm },
        { "shaft_length_mm",    in.shaft_length_mm },
        { "step_z_mm",          in.step_z_mm },
        { "step_dia_mm",        in.step_dia_mm },
        { "key_w_mm",           in.key_w_mm },
        { "key_h_mm",           in.key_h_mm },
        { "key_len_mm",         in.key_len_mm },
        { "key_center_z_mm",    in.key_center_z_mm },
        { "snap_ring_z_mm",     in.snap_ring_z_mm },
        { "snap_ring_w_mm",     in.snap_ring_w_mm },
        { "snap_ring_depth_mm", in.snap_ring_depth_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  subCount },
        { "full_dia_mm",       in.full_dia_mm },
        { "step_dia_mm",       in.step_dia_mm },
        { "step_z_mm",         in.step_z_mm },
        { "step_len_mm",       stepLen },
        { "key_w_mm",          in.key_w_mm },
        { "key_h_mm",          in.key_h_mm },
        { "key_len_mm",        in.key_len_mm },
        { "key_center_z_mm",   in.key_center_z_mm },
        { "snap_ring_z_mm",    in.snap_ring_z_mm },
        { "snap_ring_w_mm",    in.snap_ring_w_mm },
        { "snap_ring_depth_mm", in.snap_ring_depth_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type    = "lathe;end_mill;parting_tool";
    tooling.tool_dia_mm  = in.full_dia_mm;
    tooling.tool_material = "carbide";
    tooling.stock_removed_mm3 =
        M_PI * (fullR * fullR - stepR * stepR) * stepLen +
        in.key_w_mm * in.key_h_mm * in.key_len_mm +
        M_PI * (stepR * stepR -
                std::max(0.0, stepR - in.snap_ring_depth_mm) *
                std::max(0.0, stepR - in.snap_ring_depth_mm)) *
            in.snap_ring_w_mm;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "lathe" },    { "note", "turn-down step" } },
            { { "tool_type", "end_mill" }, { "tool_dia_mm", in.key_w_mm },
              { "note", "sled keyway" } },
            { { "tool_type", "parting_tool" },
              { "width_mm", in.snap_ring_w_mm },
              { "note", "DIN 471 snap-ring groove" } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::shaft_with_keyway_step applied: full Ø{} step Ø{}@z={}",
                  in.full_dia_mm, in.step_dia_mm, in.step_z_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::shaft_with_keyway_step
