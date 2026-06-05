// @lat: [[engine/skills#blade_root_tbolt_compound]]

#include "blade_root_tbolt_compound.hpp"

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

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::blade_root_tbolt_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.stud_bore_dia_mm <= 0.0 || in.stud_depth_mm <= 0.0 ||
        in.barrel_nut_dia_mm <= 0.0 || in.barrel_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "blade_root_tbolt_compound: all dims must be > 0");
        return r;
    }

    const auto* spec = thread_table::findMetric(in.stud_thread_key);
    if (!spec) {
        r.add("DFM-THREAD", "error",
              "blade_root_tbolt_compound: stud_thread_key '" +
              in.stud_thread_key + "' not in _iso_thread_table");
    } else if (in.stud_bore_dia_mm < spec->nominal_dia_mm) {
        r.add("DFM-STUD-FIT", "error",
              "blade_root_tbolt_compound: stud_bore_dia " +
              std::to_string(in.stud_bore_dia_mm) +
              " mm < thread nominal " +
              std::to_string(spec->nominal_dia_mm) + " mm");
    }

    // Barrel bore must reach the stud-bore axis to intersect it.  The cross
    // bore enters from +X edge and must travel at least to the stud center.
    if (in.barrel_depth_mm < in.stud_bore_dia_mm / 2.0) {
        r.add("DFM-INTERSECT", "error",
              "blade_root_tbolt_compound: barrel_depth " +
              std::to_string(in.barrel_depth_mm) +
              " mm too shallow to intersect stud bore (need >= " +
              std::to_string(in.stud_bore_dia_mm / 2.0) + " mm)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "blade_root_tbolt_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = thread_table::findMetric(in.stud_thread_key);
    // spec guaranteed non-null by validate.

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx   = in.axis_origin.X();
    const double cy   = in.axis_origin.Y();

    // ── 1) Axial stud bore (down from top face along +Z) ─────────────────
    const double studR = in.stud_bore_dia_mm / 2.0;
    const gp_Pnt studStart(cx, cy, topZ - in.stud_depth_mm);
    const gp_Ax2 studAx(studStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(studAx, studR, in.stud_depth_mm + kOver));

    // ── 2) Transverse barrel-nut cross bore (along +X, intersecting) ─────
    // The cross bore axis is at depth = barrel center below the top face and
    // runs along +X, passing THROUGH the stud axis (perpendicular & crossing).
    const double barrelR = in.barrel_nut_dia_mm / 2.0;
    const double crossZ  = topZ - (in.stud_depth_mm * 0.5);  // mid-stud depth
    // Start the cross bore at the +X face so it bores inward toward the axis.
    const gp_Pnt barrelStart(xMax - in.barrel_depth_mm, cy, crossZ);
    const gp_Ax2 barrelAx(barrelStart, gp_Dir(1.0, 0.0, 0.0));
    const TopoDS_Shape barrelTool = pr::cylinder(
        barrelAx, barrelR, in.barrel_depth_mm + kOver);
    current = pr::cut(current, barrelTool);

    // ── Derived volume (analytic, intersection slightly over-counts) ─────
    const double vStud   = M_PI * studR * studR * in.stud_depth_mm;
    const double vBarrel = M_PI * barrelR * barrelR * in.barrel_depth_mm;
    const double volRemoved = vStud + vBarrel;

    json params = {
        { "axis_origin",      { in.axis_origin.X(),
                                in.axis_origin.Y(),
                                in.axis_origin.Z() } },
        { "stud_bore_dia_mm", in.stud_bore_dia_mm },
        { "stud_depth_mm",    in.stud_depth_mm },
        { "barrel_nut_dia_mm",in.barrel_nut_dia_mm },
        { "barrel_depth_mm",  in.barrel_depth_mm },
        { "stud_thread_key",  in.stud_thread_key },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "wind_feature_type",          "blade_root_tbolt_joint" },
        { "subfeature_count",           2 },
        { "stud_bore_dia_mm",           in.stud_bore_dia_mm },
        { "barrel_nut_dia_mm",          in.barrel_nut_dia_mm },
        { "stud_thread_key",            in.stud_thread_key },
        { "derived_thread_nominal_mm",  spec->nominal_dia_mm },
        { "derived_thread_pitch_mm",    spec->pitch_mm },
        { "derived_cross_z_mm",         crossZ },
        { "derived_bores_perpendicular",true },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "GL/DNV T-bolt root joint" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;tap;cross_drill";
    tooling.tool_dia_mm       = in.stud_bore_dia_mm;
    tooling.tool_length_mm    = in.stud_depth_mm + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 160.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 90.0;
    tooling.extra = {
        { "wind_feature_type", "blade_root_tbolt_joint" },
        { "joint_family",      "barrel_nut" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::blade_root_tbolt_compound: stud Ø{} barrel Ø{} thread={}",
                  in.stud_bore_dia_mm, in.barrel_nut_dia_mm, in.stud_thread_key);

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
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "wind_feature_type", "blade_root_tbolt_joint" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a Z-axis cylinder (stud) and an X-axis cylinder
    // (barrel) crossing → two cylinders with perpendicular axes.
    int zCyls = 0;
    int xCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9) ++zCyls;
            else if (std::abs(d.X()) > 0.9) ++xCyls;
        } catch (...) {}
    }
    if (zCyls >= 1 && xCyls >= 1) {
        json recovered = { { "stud_thread_key", "M24" } };
        json matched   = { { "source",  "geometric_perpendicular_bores" },
                           { "z_cyls",  zCyls },
                           { "x_cyls",  xCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::blade_root_tbolt_compound
