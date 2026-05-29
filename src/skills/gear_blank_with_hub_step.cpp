// @lat: [[engine/skills#gear_blank_with_hub_step]]

#include "gear_blank_with_hub_step.hpp"

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

namespace koocadcam::skill::gear_blank_with_hub_step {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.blank_dia_mm <= 0.0 || in.blank_thick_mm <= 0.0 ||
        in.hub_dia_mm <= 0.0 || in.hub_height_mm <= 0.0 ||
        in.bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gear_blank_with_hub_step: positive blank/hub/bore required");
        return r;
    }

    if (in.bore_dia_mm < 0.8) {
        r.add("DFM-002", "error", "gear_blank_with_hub_step: bore_dia < 0.8 mm");
    }
    if (in.hub_dia_mm >= in.blank_dia_mm) {
        r.add("DFM-GEAR-HUB-DIA", "error",
              "gear_blank_with_hub_step: hub_dia " +
              std::to_string(in.hub_dia_mm) + " >= blank_dia " +
              std::to_string(in.blank_dia_mm) + " (no step)");
    }
    if (in.hub_dia_mm <= in.bore_dia_mm) {
        r.add("DFM-GEAR-WALL", "error",
              "gear_blank_with_hub_step: hub_dia <= bore_dia (no hub wall)");
    }

    const double hubWall = (in.hub_dia_mm - in.bore_dia_mm) / 2.0;
    if (hubWall < 3.0) {
        r.add("DFM-GEAR-WALL", "error",
              "gear_blank_with_hub_step: hub wall " +
              std::to_string(hubWall) + " mm < 3 mm minimum");
    }

    if (in.hub_height_mm >= in.blank_thick_mm) {
        r.add("DFM-GEAR-HUB-Z", "error",
              "gear_blank_with_hub_step: hub_height " +
              std::to_string(in.hub_height_mm) + " >= blank_thick " +
              std::to_string(in.blank_thick_mm) + " (step deeper than blank)");
    }

    // Keyway is optional but if requested both dimensions must be sane.
    if (in.keyway_w_mm > 0.0 || in.keyway_h_mm > 0.0) {
        if (in.keyway_w_mm <= 0.0 || in.keyway_h_mm <= 0.0) {
            r.add("DFM-KEYWAY-DEPTH", "error",
                  "gear_blank_with_hub_step: keyway w/h must both be > 0");
        }
        if (in.keyway_w_mm < 1.0 || in.keyway_w_mm > in.bore_dia_mm) {
            r.add("DFM-KEYWAY-FIT", "error",
                  "gear_blank_with_hub_step: keyway_w " +
                  std::to_string(in.keyway_w_mm) +
                  " mm invalid (1 ≤ key_w ≤ bore_dia)");
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
        std::string msg = "gear_blank_with_hub_step DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double blankR  = in.blank_dia_mm / 2.0;
    const double hubR    = in.hub_dia_mm   / 2.0;
    const double boreR   = in.bore_dia_mm  / 2.0;
    const double Z       = in.blank_thick_mm;
    const double hubH    = in.hub_height_mm;
    const double kOverhang = 0.05;

    // ── Tool 1: hub step.  Remove an annular ring (outerR = blankR,
    //            innerR = hubR) from the top hub_height of the blank.
    const double stepZ0 = Z - hubH;
    const gp_Ax2 axStep(gp_Pnt(0, 0, stepZ0), gp::DZ());
    const TopoDS_Shape stepTool =
        pr::annularRing(axStep, blankR + kOverhang, hubR, hubH + kOverhang);

    // ── Tool 2: through bore
    const gp_Ax2 axBore(gp_Pnt(0, 0, -kOverhang), gp::DZ());
    const TopoDS_Shape boreTool =
        pr::cylinder(axBore, boreR, Z + 2.0 * kOverhang);

    // ── Tool 3 (optional): keyway slot in bore (axial through)
    bool hasKey = (in.keyway_w_mm > 0.0 && in.keyway_h_mm > 0.0);
    TopoDS_Shape keyBox;
    if (hasKey) {
        const double kw = in.keyway_w_mm;
        const double kh = in.keyway_h_mm;
        const gp_Pnt keyOrigin(-kw / 2.0, boreR, -kOverhang);
        const gp_Ax2 keyAx(keyOrigin, gp::DZ());
        keyBox = pr::box(keyAx, kw, kh, Z + 2.0 * kOverhang);
    }

    // Fuse bore + keyway (coaxial overlap) then compound cut with step ring.
    TopoDS_Shape boreFused = hasKey ? pr::fuse(boreTool, keyBox) : boreTool;
    std::vector<TopoDS_Shape> tools = { stepTool, boreFused };
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    const int subCount = 2 + (hasKey ? 1 : 0);

    json params = {
        { "blank_dia_mm",   in.blank_dia_mm },
        { "blank_thick_mm", in.blank_thick_mm },
        { "hub_dia_mm",     in.hub_dia_mm },
        { "hub_height_mm",  in.hub_height_mm },
        { "bore_dia_mm",    in.bore_dia_mm },
        { "keyway_w_mm",    in.keyway_w_mm },
        { "keyway_h_mm",    in.keyway_h_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  subCount },
        { "blank_dia_mm",      in.blank_dia_mm },
        { "blank_thick_mm",    in.blank_thick_mm },
        { "hub_dia_mm",        in.hub_dia_mm },
        { "hub_height_mm",     in.hub_height_mm },
        { "bore_dia_mm",       in.bore_dia_mm },
        { "hub_wall_mm",       (in.hub_dia_mm - in.bore_dia_mm) / 2.0 },
        { "has_keyway",        hasKey },
        { "keyway_w_mm",       in.keyway_w_mm },
        { "keyway_h_mm",       in.keyway_h_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type    = "lathe;drill;end_mill";
    tooling.tool_dia_mm  = in.bore_dia_mm;
    tooling.tool_material = "carbide";
    tooling.stock_removed_mm3 =
        M_PI * (blankR * blankR - hubR * hubR) * hubH +
        M_PI * boreR * boreR * Z +
        (hasKey ? in.keyway_w_mm * in.keyway_h_mm * Z : 0.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "lathe" },    { "note", "turn-down hub step" } },
            { { "tool_type", "drill" },    { "tool_dia_mm", in.bore_dia_mm } },
            { { "tool_type", "end_mill" }, { "tool_dia_mm", in.keyway_w_mm },
              { "note", "keyway in bore (skipped if w=0)" } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::gear_blank_with_hub_step applied: blank Ø{} hub Ø{}×{} bore Ø{}",
                  in.blank_dia_mm, in.hub_dia_mm, in.hub_height_mm, in.bore_dia_mm);

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

}  // namespace koocadcam::skill::gear_blank_with_hub_step
