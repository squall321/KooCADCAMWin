// @lat: [[engine/skills#pipe_clamp_saddle_compound]]

#include "pipe_clamp_saddle_compound.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pipe_clamp_saddle_compound {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pipe_dia_mm <= 0.0) {
        r.add("DFM-PIPE-DIA", "error",
              "pipe_clamp_saddle_compound: pipe_dia must be > 0");
    }
    if (in.saddle_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pipe_clamp_saddle_compound: saddle_thickness must be > 0");
    }
    if (in.pipe_dia_mm > 0.0
        && in.saddle_thickness_mm < 0.5 * in.pipe_dia_mm) {
        r.add("DFM-SADDLE-THK", "error",
              "pipe_clamp_saddle_compound: saddle_thickness " +
              std::to_string(in.saddle_thickness_mm) +
              " mm < 0.5 × pipe_dia (need ≥ " +
              std::to_string(0.5 * in.pipe_dia_mm) + " mm)");
    }
    if (in.bolt_thread_size_key.empty()) {
        r.add("DFM-M-THREAD", "error",
              "pipe_clamp_saddle_compound: bolt_thread_size_key is empty");
    } else {
        const tt::MetricThreadSpec* bs = tt::findMetric(in.bolt_thread_size_key);
        if (!bs) {
            r.add("DFM-M-THREAD", "error",
                  "pipe_clamp_saddle_compound: unknown bolt thread '" +
                  in.bolt_thread_size_key + "' (must be in central M-thread table)");
        } else {
            // Bolt centres must clear pipe saddle radius + bolt clearance/2.
            const double pipeR = in.pipe_dia_mm / 2.0;
            const double boltClearR = bs->clearance_medium_mm / 2.0;
            const double minLat = pipeR + boltClearR + 1.0;
            const double rA = std::sqrt(in.bolt_a_xy[0] * in.bolt_a_xy[0]
                                      + in.bolt_a_xy[1] * in.bolt_a_xy[1]);
            if (rA < minLat) {
                r.add("DFM-BOLT-SPACE", "error",
                      "pipe_clamp_saddle_compound: bolt A too close to pipe saddle ("
                      + std::to_string(rA) + " < " + std::to_string(minLat) + ")");
            }
            const double rB = std::sqrt(in.bolt_b_xy[0] * in.bolt_b_xy[0]
                                      + in.bolt_b_xy[1] * in.bolt_b_xy[1]);
            if (rB < minLat) {
                r.add("DFM-BOLT-SPACE", "error",
                      "pipe_clamp_saddle_compound: bolt B too close to pipe saddle ("
                      + std::to_string(rB) + " < " + std::to_string(minLat) + ")");
            }
        }
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pipe_clamp_saddle_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* bs = tt::findMetric(in.bolt_thread_size_key);
    if (!bs) throw SkillError("pipe_clamp_saddle_compound: bolt lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double zTop = zMax;
    const double pipeR = in.pipe_dia_mm / 2.0;
    const double saddleR = pipeR + 0.3;       // small clearance fit

    // Pipe axis runs along the (validated) +X direction at z = zTop +
    // pipe_axis_origin.Z() but for the saddle pocket we anchor the
    // saddle cylinder along the bolt-bracket's TOP face, so:
    //  - the centre of the cut cylinder sits at z = zTop (so the cylinder
    //    spans down by pipeR — full diameter exposed on top).
    const gp_Pnt pipeCenter(
        (xMin + xMax) / 2.0 + in.pipe_axis_origin.X(),
        (yMin + yMax) / 2.0 + in.pipe_axis_origin.Y(),
        zTop);

    // ── 1) saddle pocket: full cylinder cut sized to pipe OD ───────────────
    // The cylinder is centred on the top face Z so only the lower half is
    // intersected with the bracket → produces a semi-cyl saddle pocket.
    // Cut cylinder runs along +X (in.pipe_axis_dir).
    const double saddleLen = (xMax - xMin) + 2.0 * kOver;
    const gp_Pnt cutStart(xMin - kOver, pipeCenter.Y(), pipeCenter.Z());
    const gp_Ax2 saddleAx(cutStart, in.pipe_axis_dir);
    const TopoDS_Shape saddleTool = pr::cylinder(saddleAx, saddleR, saddleLen);

    TopoDS_Shape current = pr::cut(wp.shape(), saddleTool);

    // ── 2) bolt A clearance hole through full thickness ───────────────────
    const double boltClearR = bs->clearance_medium_mm / 2.0;
    const double thickness  = (zMax - zMin) + 2.0 * kOver;
    {
        const gp_Pnt p(in.bolt_a_xy[0], in.bolt_a_xy[1], zMin - kOver);
        const gp_Ax2 ax(p, gp::DZ());
        const TopoDS_Shape tool = pr::cylinder(ax, boltClearR, thickness);
        current = pr::cut(current, tool);
    }
    // ── 3) bolt B clearance hole ──────────────────────────────────────────
    {
        const gp_Pnt p(in.bolt_b_xy[0], in.bolt_b_xy[1], zMin - kOver);
        const gp_Ax2 ax(p, gp::DZ());
        const TopoDS_Shape tool = pr::cylinder(ax, boltClearR, thickness);
        current = pr::cut(current, tool);
    }

    const double vSaddle = 0.5 * M_PI * saddleR * saddleR * (xMax - xMin);  // half-cyl
    const double vBolt   = M_PI * boltClearR * boltClearR * (zMax - zMin);
    const double volRemoved = vSaddle + 2.0 * vBolt;

    json params = {
        { "pipe_axis_origin",       { in.pipe_axis_origin.X(),
                                      in.pipe_axis_origin.Y(),
                                      in.pipe_axis_origin.Z() } },
        { "pipe_axis_dir",          { in.pipe_axis_dir.X(),
                                      in.pipe_axis_dir.Y(),
                                      in.pipe_axis_dir.Z() } },
        { "pipe_dia_mm",            in.pipe_dia_mm },
        { "saddle_thickness_mm",    in.saddle_thickness_mm },
        { "bolt_a_xy",              { in.bolt_a_xy[0], in.bolt_a_xy[1] } },
        { "bolt_b_xy",              { in.bolt_b_xy[0], in.bolt_b_xy[1] } },
        { "bolt_thread_size_key",   in.bolt_thread_size_key },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "hvac_feature_type",      "pipe_clamp_saddle" },
        { "subfeature_count",       3 },          // saddle + 2 bolts
        { "pipe_dia_mm",            in.pipe_dia_mm },
        { "bolt_count",             2 },
        { "bolt_thread_size_key",   in.bolt_thread_size_key },
        { "bolt_clearance_dia_mm",  bs->clearance_medium_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "ball_mill;drill;drill";
    tooling.tool_dia_mm       = bs->clearance_medium_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 15.0;
    tooling.extra = {
        { "standard",  "ANSI/MSS SP-58 pipe clamp" },
        { "bolt_M",    in.bolt_thread_size_key },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pipe_clamp_saddle_compound: pipe_dia={} bolt={}",
                  in.pipe_dia_mm, in.bolt_thread_size_key);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric: find 1 large +X-axis half-cylinder + 2 small +Z bolt cyls.
    int saddleCyls = 0;
    int boltCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.X()) > 0.9) ++saddleCyls;
            else if (std::abs(d.Z()) > 0.9) ++boltCyls;
        } catch (...) {}
    }
    if (saddleCyls >= 1 && boltCyls >= 2) {
        json recovered = { { "bolt_count", boltCyls } };
        json matched   = { { "source", "geometric_saddle_pattern" },
                           { "saddle_cyl_count", saddleCyls },
                           { "bolt_cyl_count",   boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::pipe_clamp_saddle_compound
