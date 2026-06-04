// @lat: [[engine/skills#beam_dump_aperture_blackbody]]

#include "beam_dump_aperture_blackbody.hpp"

#include "Workpiece.hpp"
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
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::beam_dump_aperture_blackbody {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "beam_dump_aperture_blackbody: face_id out of range");
    }
    if (!(in.aperture_dia_mm > 0.0) || !(in.aperture_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "beam_dump_aperture_blackbody: aperture_dia/depth must be > 0");
    }
    if (!(in.internal_chamber_dia_mm > 0.0) ||
        !(in.internal_chamber_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "beam_dump_aperture_blackbody: chamber dia/depth must be > 0");
    }
    if (in.aperture_dia_mm > 0.0 && in.internal_chamber_dia_mm > 0.0 &&
        in.aperture_dia_mm >= in.internal_chamber_dia_mm) {
        r.add("DFM-APERTURE", "error",
              "beam_dump_aperture_blackbody: aperture_dia ("
              + std::to_string(in.aperture_dia_mm)
              + ") must be < internal_chamber_dia ("
              + std::to_string(in.internal_chamber_dia_mm) + ")");
    }
    if (in.cone_angle_deg < 5.0 || in.cone_angle_deg > 30.0) {
        r.add("DFM-CONE-ANGLE", "error",
              "beam_dump_aperture_blackbody: cone_angle_deg "
              + std::to_string(in.cone_angle_deg)
              + " out of range [5, 30]");
    }
    if (in.face_id >= 0 && in.face_id < wp.faceCount()) {
        if (!wp.isFacePlanar(in.face_id)) {
            r.add("DFM-FACE", "error",
                  "beam_dump_aperture_blackbody: target face must be planar");
        }
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "beam_dump_aperture_blackbody DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();
    const double cx    = in.center_x_mm;
    const double cy    = in.center_y_mm;
    constexpr double kEmbed = 0.05;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(3);

    // Op 1: aperture entry (small cylinder from top face, depth =
    // aperture_depth_mm).
    const double apR = in.aperture_dia_mm / 2.0;
    {
        const gp_Pnt c(cx, cy, baseZ - in.aperture_depth_mm);
        const gp_Ax2 ax(c, gp::DZ());
        cutters.push_back(pr::cylinder(ax, apR, in.aperture_depth_mm + kEmbed));
    }

    // Op 2: internal chamber (larger cylinder, beginning where aperture ends).
    const double chR = in.internal_chamber_dia_mm / 2.0;
    const double chTopZ = baseZ - in.aperture_depth_mm;
    const double chBotZ = chTopZ - in.internal_chamber_depth_mm;
    {
        const gp_Pnt c(cx, cy, chBotZ);
        const gp_Ax2 ax(c, gp::DZ());
        cutters.push_back(pr::cylinder(ax, chR, in.internal_chamber_depth_mm + kEmbed));
    }

    // Op 3: conical end — cone from chamber radius down to a small tip apex,
    // tip height computed from cone_angle_deg (half-angle).
    const double halfAngleRad = in.cone_angle_deg * M_PI / 180.0;
    const double coneTipHeight = chR / std::tan(halfAngleRad);
    {
        const gp_Pnt apex(cx, cy, chBotZ - coneTipHeight);
        const gp_Ax2 ax(apex, gp::DZ());
        // coneFrustum(ax, r1, r2, height): builds from r1 at ax.Location()
        // to r2 at distance `height` along ax.Direction().
        // Here r1 = ~0 (tip), r2 = chR (mouth at chBotZ).
        cutters.push_back(pr::coneFrustum(ax, 0.05, chR, coneTipHeight + kEmbed));
    }

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::cutMany(wp.shape(), cutters);
    if (newShape.IsNull())
        throw SkillError("beam_dump_aperture_blackbody: cutMany returned null");
    const double volRemoved = volBefore - volumeOf(newShape);

    const double vAp   = M_PI * apR * apR * in.aperture_depth_mm;
    const double vCh   = M_PI * chR * chR * in.internal_chamber_depth_mm;
    const double vCone = (1.0 / 3.0) * M_PI * chR * chR * coneTipHeight;

    json params = {
        { "face_id",                   in.face_id },
        { "center_x_mm",               in.center_x_mm },
        { "center_y_mm",               in.center_y_mm },
        { "aperture_dia_mm",           in.aperture_dia_mm },
        { "aperture_depth_mm",         in.aperture_depth_mm },
        { "internal_chamber_dia_mm",   in.internal_chamber_dia_mm },
        { "internal_chamber_depth_mm", in.internal_chamber_depth_mm },
        { "cone_angle_deg",            in.cone_angle_deg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "optical_feature_type",       "beam_dump_blackbody" },
        { "subfeature_count",           3 },
        { "has_conical_chamber",        true },
        { "aperture_dia_mm",            in.aperture_dia_mm },
        { "internal_chamber_dia_mm",    in.internal_chamber_dia_mm },
        { "cone_angle_deg",             in.cone_angle_deg },
        { "cone_tip_height_mm",         coneTipHeight },
        { "derived_volume_removed_mm3", volRemoved },
        { "estimated_aperture_mm3",     vAp },
        { "estimated_chamber_mm3",      vCh },
        { "estimated_cone_mm3",         vCone },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill;bore;chamfer";
    tooling.tool_dia_mm      = in.internal_chamber_dia_mm;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 60.0 + 15.0;
    tooling.extra = {
        { "spec_source", "beam dump / black-body absorber geometry" },
        { "internal_finish", "anodize_matte_black" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);
    spdlog::debug("skill::beam_dump_aperture_blackbody: ap={} ch={} cone_deg={}",
                  in.aperture_dia_mm, in.internal_chamber_dia_mm, in.cone_angle_deg);
    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.93;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::beam_dump_aperture_blackbody
