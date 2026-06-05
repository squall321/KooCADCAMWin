// @lat: [[engine/skills#htd_timing_pulley_teeth]]

#include "htd_timing_pulley_teeth.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::htd_timing_pulley_teeth {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

bool isStandardPitch(double p)
{
    return std::abs(p - 3.0) < 1e-6 ||
           std::abs(p - 5.0) < 1e-6 ||
           std::abs(p - 8.0) < 1e-6;
}
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.htd_pitch_mm <= 0.0 || in.belt_width_mm <= 0.0 ||
        in.tooth_depth_mm <= 0.0 || in.blank_outer_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "htd_timing_pulley_teeth: all dimensions must be > 0");
        return r;
    }

    if (!isStandardPitch(in.htd_pitch_mm)) {
        r.add("DFM-PT-PITCH", "error",
              "htd_timing_pulley_teeth: htd_pitch_mm (" +
              std::to_string(in.htd_pitch_mm) +
              ") must be one of {3, 5, 8}");
    }

    if (in.tooth_count < 10) {
        r.add("DFM-PT-TEETH", "error",
              "htd_timing_pulley_teeth: tooth_count (" +
              std::to_string(in.tooth_count) +
              ") must be >= 10");
    }

    if (in.tooth_count >= 1) {
        const double pcd = (in.htd_pitch_mm *
                            static_cast<double>(in.tooth_count)) / M_PI;
        if (in.blank_outer_dia_mm <= pcd) {
            r.add("DFM-PT-BLANK", "error",
                  "htd_timing_pulley_teeth: blank_outer_dia_mm (" +
                  std::to_string(in.blank_outer_dia_mm) +
                  ") must exceed pitch circle diameter (" +
                  std::to_string(pcd) + ")");
        }
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "htd_timing_pulley_teeth DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;

    const double odR = in.blank_outer_dia_mm / 2.0;
    // Tooth gap rounded pocket radius scaled to the pitch (HTD curvilinear
    // root): gap width roughly 0.6·pitch.
    const double gapR = 0.30 * in.htd_pitch_mm;
    // Center the rounded pocket so it bites tooth_depth into the OD.
    const double gapCenterR = odR - in.tooth_depth_mm + gapR;

    const gp_Pnt gapTpl(cx + gapCenterR, cy, zMin - kOver);
    const gp_Ax2 gapAx(gapTpl, gp::DZ());
    const TopoDS_Shape gapTemplate = pr::cylinder(gapAx, gapR, thru);

    const gp_Ax1 rotAxis(gp_Pnt(cx, cy, zMax), gp::DZ());
    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < in.tooth_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.tooth_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(gapTemplate, rot, true);
        if (!xform.IsDone())
            throw SkillError("htd_timing_pulley_teeth: tooth rotation failed");
        current = pr::cut(current, xform.Shape());  // sequential — no compound
    }

    const double diskThk = (zMax - zMin);
    const double vGap    = M_PI * gapR * gapR * diskThk;
    const double volRemoved = static_cast<double>(in.tooth_count) * vGap;
    const double pcd = (in.htd_pitch_mm *
                        static_cast<double>(in.tooth_count)) / M_PI;

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "htd_pitch_mm",       in.htd_pitch_mm },
        { "tooth_count",        in.tooth_count },
        { "belt_width_mm",      in.belt_width_mm },
        { "tooth_depth_mm",     in.tooth_depth_mm },
        { "blank_outer_dia_mm", in.blank_outer_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "powertrans_feature_type",    "htd_timing_pulley_teeth" },
        { "subfeature_count",           in.tooth_count },
        { "htd_pitch_mm",               in.htd_pitch_mm },
        { "tooth_count",                in.tooth_count },
        { "derived_pitch_circle_dia_mm", pcd },
        { "derived_tooth_gap_radius_mm", gapR },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "HTD / GT curvilinear tooth profile" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "hob;form_mill";
    tooling.tool_dia_mm       = 2.0 * gapR;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 5.0 * static_cast<double>(in.tooth_count);
    tooling.extra = {
        { "powertrans_application", "htd_timing_pulley" },
        { "htd_pitch_mm",          in.htd_pitch_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::htd_timing_pulley_teeth: pitch={} N={} faces {}→{}",
                  in.htd_pitch_mm, in.tooth_count,
                  wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: count vertical cylindrical tooth-gap pockets.
    int gapCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 0.5 && radius <= 4.0) ++gapCyls;
        } catch (...) {}
    }
    if (gapCyls >= 10) {
        json recovered = { { "tooth_count",  gapCyls },
                           { "htd_pitch_mm", 5.0 } };
        json matched   = { { "source",    "geometric_tooth_gap_ring" },
                           { "gap_cyls",  gapCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::htd_timing_pulley_teeth
