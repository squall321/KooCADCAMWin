// @lat: [[engine/skills#optical_axis_alignment_pin_pair]]

#include "optical_axis_alignment_pin_pair.hpp"

#include "_iso286_fits.hpp"
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

namespace koocadcam::skill::optical_axis_alignment_pin_pair {

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
              "optical_axis_alignment_pin_pair: face_id out of range");
    }
    if (!(in.pin_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "optical_axis_alignment_pin_pair: pin_dia_mm must be > 0");
    }
    if (!(in.pin_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "optical_axis_alignment_pin_pair: pin_depth_mm must be > 0");
    }
    if (in.tolerance_class != "H6" && in.tolerance_class != "H7") {
        r.add("DFM-PIN-TOL", "error",
              "optical_axis_alignment_pin_pair: tolerance_class must be H6 or H7, got '"
              + in.tolerance_class + "'");
    }
    if (in.pin_dia_mm > 0.0) {
        const iso286::FitBand* b = iso286::findBand(in.pin_dia_mm);
        if (!b) {
            r.add("DFM-PIN-DIA", "error",
                  "optical_axis_alignment_pin_pair: pin_dia_mm "
                  + std::to_string(in.pin_dia_mm)
                  + " out of ISO 286 H6/H7 lookup range (≤ 100 mm)");
        }
    }
    const double dx = in.pin_b_x_mm - in.pin_a_x_mm;
    const double dy = in.pin_b_y_mm - in.pin_a_y_mm;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (in.pin_dia_mm > 0.0 && dist < 2.0 * in.pin_dia_mm) {
        r.add("DFM-PIN-SPACING", "error",
              "optical_axis_alignment_pin_pair: pin centres too close ("
              + std::to_string(dist) + " mm < 2×dia "
              + std::to_string(2.0 * in.pin_dia_mm) + " mm)");
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "optical_axis_alignment_pin_pair DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();
    constexpr double kEmbed = 0.05;

    const iso286::FitBand* band = iso286::findBand(in.pin_dia_mm);
    const double devUm = (in.tolerance_class == "H6")
        ? static_cast<double>(band->h6_dev_um)
        : static_cast<double>(band->h7_dev_um);
    const double finalDia = in.pin_dia_mm + devUm * 1e-3 / 2.0;
    const double finalR   = finalDia / 2.0;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(2);

    auto buildPinHole = [&](double cx, double cy) {
        const gp_Pnt c(cx, cy, baseZ - in.pin_depth_mm);
        const gp_Ax2 ax(c, gp::DZ());
        return pr::cylinder(ax, finalR, in.pin_depth_mm + kEmbed);
    };
    cutters.push_back(buildPinHole(in.pin_a_x_mm, in.pin_a_y_mm));
    cutters.push_back(buildPinHole(in.pin_b_x_mm, in.pin_b_y_mm));

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::cutMany(wp.shape(), cutters);
    if (newShape.IsNull())
        throw SkillError("optical_axis_alignment_pin_pair: cutMany returned null");
    const double volRemoved = volBefore - volumeOf(newShape);

    json params = {
        { "face_id",         in.face_id },
        { "pin_a_x_mm",      in.pin_a_x_mm },
        { "pin_a_y_mm",      in.pin_a_y_mm },
        { "pin_b_x_mm",      in.pin_b_x_mm },
        { "pin_b_y_mm",      in.pin_b_y_mm },
        { "pin_dia_mm",      in.pin_dia_mm },
        { "pin_depth_mm",    in.pin_depth_mm },
        { "tolerance_class", in.tolerance_class },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "optical_feature_type",       "alignment_pin_pair" },
        { "subfeature_count",           2 },
        { "tolerance_class",            in.tolerance_class },
        { "pin_dia_mm",                 in.pin_dia_mm },
        { "final_dia_mm",               finalDia },
        { "pin_depth_mm",               in.pin_depth_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill;reamer";
    tooling.tool_dia_mm      = finalDia;
    tooling.tool_material    = "hss-cobalt";
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 2.0 * (in.pin_depth_mm / 10.0 + 4.0);
    tooling.extra = {
        { "spec_source", "ISO 286-1:2010 (H6/H7)" },
        { "mating_part", "DIN 7 / ISO 2338 cylindrical dowel pin" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);
    spdlog::debug("skill::optical_axis_alignment_pin_pair: dia={} tol={}",
                  in.pin_dia_mm, in.tolerance_class);
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
        rf.confidence       = 0.94;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::optical_axis_alignment_pin_pair
