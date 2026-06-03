// @lat: [[engine/skills#runner_balanced_h]]

#include "runner_balanced_h.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::runner_balanced_h {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.leg_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "runner_balanced_h: leg_length_mm must be > 0");
    }
    if (!(in.runner_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "runner_balanced_h: runner_dia_mm must be > 0");
    } else if (in.runner_dia_mm < 2.0 || in.runner_dia_mm > 8.0) {
        r.add("DFM-RUNNER-DIA", "error",
              "runner_balanced_h: runner_dia " +
              std::to_string(in.runner_dia_mm) +
              " mm outside [2.0, 8.0] mm typical");
    }
    if (!(in.runner_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "runner_balanced_h: runner_depth_mm must be > 0");
    } else if (in.runner_depth_mm < 0.5) {
        r.add("DFM-RUNNER-DEPTH", "error",
              "runner_balanced_h: runner_depth " +
              std::to_string(in.runner_depth_mm) +
              " < 0.5 mm clearance minimum");
    }
    if (in.cavity_count != 4 && in.cavity_count != 8) {
        r.add("DFM-RUNNER-CAVITY", "error",
              "runner_balanced_h: cavity_count " +
              std::to_string(in.cavity_count) +
              " must be 4 or 8");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "runner_balanced_h DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Workpiece bbox to find the parting-plane top.
    const auto bb = pr::optimalBbox(wp.shape());
    const double topZ = bb.zMax;

    const double r       = in.runner_dia_mm / 2.0;
    const double depth   = in.runner_depth_mm;
    const double L       = in.leg_length_mm;
    constexpr double kOverhang = 0.05;
    const gp_Dir downward(0.0, 0.0, -1.0);

    std::vector<TopoDS_Shape> cutters;

    // 1. Sprue — vertical cylinder at sprue_xy.
    {
        const gp_Pnt top(in.sprue_x_mm, in.sprue_y_mm, topZ + kOverhang);
        const gp_Ax2 ax(top, downward);
        cutters.push_back(pr::cylinder(ax, r, depth + kOverhang));
    }

    // 2. Primary legs — along ±X at the sprue runner depth.
    //    Implemented as horizontal cylinders just below the parting plane
    //    (depth below topZ → centred at topZ - depth/2).  For visual
    //    correctness in the geometry we just sink them centred under topZ
    //    at depth − dia/2.
    const double legZ = topZ - depth + r;        // axis Z so leg lies INSIDE
    const double legZc = std::min(legZ, topZ - 1e-3);

    // primary +X
    {
        const gp_Pnt start(in.sprue_x_mm, in.sprue_y_mm, legZc);
        const gp_Ax2 ax(start, gp_Dir(1.0, 0.0, 0.0));
        cutters.push_back(pr::cylinder(ax, r, L));
    }
    // primary -X
    {
        const gp_Pnt start(in.sprue_x_mm, in.sprue_y_mm, legZc);
        const gp_Ax2 ax(start, gp_Dir(-1.0, 0.0, 0.0));
        cutters.push_back(pr::cylinder(ax, r, L));
    }

    // 3. Secondary legs — 4 of them, perpendicular at primary tips.
    const double tipX[2] = { in.sprue_x_mm + L, in.sprue_x_mm - L };
    for (double tx : tipX) {
        // +Y
        {
            const gp_Pnt start(tx, in.sprue_y_mm, legZc);
            const gp_Ax2 ax(start, gp_Dir(0.0, 1.0, 0.0));
            cutters.push_back(pr::cylinder(ax, r, L));
        }
        // -Y
        {
            const gp_Pnt start(tx, in.sprue_y_mm, legZc);
            const gp_Ax2 ax(start, gp_Dir(0.0, -1.0, 0.0));
            cutters.push_back(pr::cylinder(ax, r, L));
        }
    }

    // 4. Tertiary legs (cavity_count == 8) — 8 short legs at secondary tips,
    //    along ±X at length L/2.
    if (in.cavity_count == 8) {
        const double L2 = L / 2.0;
        for (double tx : tipX) {
            for (double ty : { in.sprue_y_mm + L, in.sprue_y_mm - L }) {
                // +X
                {
                    const gp_Pnt start(tx, ty, legZc);
                    const gp_Ax2 ax(start, gp_Dir(1.0, 0.0, 0.0));
                    cutters.push_back(pr::cylinder(ax, r, L2));
                }
                // -X
                {
                    const gp_Pnt start(tx, ty, legZc);
                    const gp_Ax2 ax(start, gp_Dir(-1.0, 0.0, 0.0));
                    cutters.push_back(pr::cylinder(ax, r, L2));
                }
            }
        }
    }

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::cutMany(wp.shape(), cutters);
    if (newShape.IsNull()) {
        throw SkillError("runner_balanced_h: cutMany returned null shape");
    }
    const double volAfter  = volumeOf(newShape);
    const double removed   = volBefore - volAfter;

    json params = {
        { "sprue_x_mm",      in.sprue_x_mm },
        { "sprue_y_mm",      in.sprue_y_mm },
        { "leg_length_mm",   in.leg_length_mm },
        { "runner_dia_mm",   in.runner_dia_mm },
        { "runner_depth_mm", in.runner_depth_mm },
        { "cavity_count",    in.cavity_count },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "mold_feature_type",       "balanced_h_runner" },
        { "derived_volume_removed",  removed },
        { "cutter_count",            static_cast<int>(cutters.size()) },
        { "volume_before_mm3",       volBefore },
        { "volume_after_mm3",        volAfter },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "mold_runner";
    tooling.tool_dia_mm       = in.runner_dia_mm;
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::runner_balanced_h applied: cavities={} cutters={} removed={}",
                  in.cavity_count, cutters.size(), removed);

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
        rf.confidence       = 0.92;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::runner_balanced_h
