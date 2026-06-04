// @lat: [[engine/skills#iris_aperture_slot_array]]

#include "iris_aperture_slot_array.hpp"

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

namespace koocadcam::skill::iris_aperture_slot_array {

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
              "iris_aperture_slot_array: face_id out of range");
    }
    if (!(in.bore_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "iris_aperture_slot_array: bore_dia_mm must be > 0");
    }
    if (!(in.slot_width_mm > 0.0) || !(in.slot_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "iris_aperture_slot_array: slot_width_mm and slot_depth_mm must be > 0");
    }
    if (in.slot_count < 4 || in.slot_count > 36) {
        r.add("DFM-SLOT-COUNT", "error",
              "iris_aperture_slot_array: slot_count " + std::to_string(in.slot_count)
              + " out of supported range [4, 36]");
    }
    if (in.slot_outer_radius_mm <= in.slot_inner_radius_mm) {
        r.add("DFM-SLOT-RADIUS", "error",
              "iris_aperture_slot_array: slot_outer_radius_mm must be > slot_inner_radius_mm");
    }
    if (in.bore_dia_mm > 0.0 &&
        in.slot_inner_radius_mm < in.bore_dia_mm / 2.0 - 1e-6) {
        r.add("DFM-SLOT-RADIUS", "error",
              "iris_aperture_slot_array: slot_inner_radius_mm must be >= bore_dia/2");
    }
    if (in.face_id >= 0 && in.face_id < wp.faceCount()) {
        if (!wp.isFacePlanar(in.face_id)) {
            r.add("DFM-FACE", "error",
                  "iris_aperture_slot_array: target face must be planar");
        }
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "iris_aperture_slot_array DFM failed:";
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
    cutters.reserve(static_cast<size_t>(1 + in.slot_count));

    // Op 1: central bore (through-ish — uses slot_depth as bore depth too).
    {
        const gp_Pnt c(cx, cy, baseZ - in.slot_depth_mm);
        const gp_Ax2 ax(c, gp::DZ());
        cutters.push_back(pr::cylinder(ax, in.bore_dia_mm / 2.0,
                                       in.slot_depth_mm + kEmbed));
    }

    // Op 2..N+1: evenly-spaced thin angular slot tools.
    //
    // Each slot is approximated as a small rectangular pocket of size
    // (radial_len × slot_width × slot_depth), centred at radius
    // R = (slot_outer_radius + slot_inner_radius)/2 and angle θ.  Cheap, robust
    // model — fits the cosmetic intent for analyzability without arc geometry.
    const double Rc = 0.5 * (in.slot_outer_radius_mm + in.slot_inner_radius_mm);
    const double radialLen = in.slot_outer_radius_mm - in.slot_inner_radius_mm;
    const double W = in.slot_width_mm;
    const double D = in.slot_depth_mm;

    for (int i = 0; i < in.slot_count; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i)
                             / static_cast<double>(in.slot_count);
        const double sxC = cx + Rc * std::cos(theta);
        const double syC = cy + Rc * std::sin(theta);
        try {
            // Build a tiny box oriented along the radial direction.  Tangent
            // axis = (-sin, cos); radial axis = (cos, sin).
            // Place the box centred at the slot midpoint.
            const double half_r = radialLen / 2.0;
            const double half_t = W / 2.0;
            const double cosT = std::cos(theta);
            const double sinT = std::sin(theta);
            // Corner of the slot box at (sxC - half_r*radial - half_t*tangent),
            // baseZ - D.
            const double ox = sxC - cosT * half_r - (-sinT) * half_t;
            const double oy = syC - sinT * half_r -  ( cosT) * half_t;
            const gp_Pnt origin(ox, oy, baseZ - D);
            const gp_Ax2 ax(origin, gp_Dir(0, 0, 1), gp_Dir(cosT, sinT, 0.0));
            cutters.push_back(pr::box(ax, radialLen, W, D + 2.0 * kEmbed));
        } catch (const Standard_Failure& ex) {
            spdlog::debug("iris_aperture_slot_array: slot {} failed — {}",
                          i, ex.what());
        }
    }

    if (cutters.size() <= 1) {
        throw SkillError("iris_aperture_slot_array: no slots produced");
    }

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::cutMany(wp.shape(), cutters);
    if (newShape.IsNull())
        throw SkillError("iris_aperture_slot_array: cutMany returned null");
    const double volRemoved = volBefore - volumeOf(newShape);

    const double estSlotVol = radialLen * W * D
                              * static_cast<double>(in.slot_count);
    const double estBoreVol = M_PI * std::pow(in.bore_dia_mm / 2.0, 2.0) * D;

    json params = {
        { "face_id",              in.face_id },
        { "center_x_mm",          in.center_x_mm },
        { "center_y_mm",          in.center_y_mm },
        { "bore_dia_mm",          in.bore_dia_mm },
        { "slot_count",           in.slot_count },
        { "slot_inner_radius_mm", in.slot_inner_radius_mm },
        { "slot_outer_radius_mm", in.slot_outer_radius_mm },
        { "slot_width_mm",        in.slot_width_mm },
        { "slot_depth_mm",        in.slot_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "is_periodic_pattern",        true },
        { "optical_feature_type",       "iris_slot_array" },
        { "subfeature_count",           static_cast<int>(cutters.size()) },
        { "slot_count",                 in.slot_count },
        { "bore_dia_mm",                in.bore_dia_mm },
        { "slot_width_mm",              in.slot_width_mm },
        { "slot_inner_radius_mm",       in.slot_inner_radius_mm },
        { "slot_outer_radius_mm",       in.slot_outer_radius_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "estimated_slot_volume_mm3",  estSlotVol },
        { "estimated_bore_volume_mm3",  estBoreVol },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill;end_mill";
    tooling.tool_dia_mm      = std::max(0.3, in.slot_width_mm * 0.7);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 50.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);
    spdlog::debug("skill::iris_aperture_slot_array: N={} bore={} Rc={}",
                  in.slot_count, in.bore_dia_mm, Rc);
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

}  // namespace koocadcam::skill::iris_aperture_slot_array
