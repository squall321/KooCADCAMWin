// @lat: [[engine/skills#ag_implement_3point_hitch_pin]]

#include "ag_implement_3point_hitch_pin.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::ag_implement_3point_hitch_pin {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr double kOver = 0.1;

double pinDiameterForCategory(int cat)
{
    // ASAE S217 / ISO 730 pin diameters per category.
    switch (cat) {
        case 1: return 19.0;   // 3/4"
        case 2: return 25.4;   // 1"
        case 3: return 31.8;   // 1-1/4"
        default: return -1.0;
    }
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.linch_pin_hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ag_implement_3point_hitch_pin: linch_pin_hole_dia must be > 0");
    }
    if (in.hitch_category < 1 || in.hitch_category > 3) {
        r.add("DFM-CAT", "error",
              "ag_implement_3point_hitch_pin: hitch_category " +
              std::to_string(in.hitch_category) +
              " invalid (must be 1, 2, or 3)");
    }
    if (in.pin_position != "top_link" && in.pin_position != "lower_link") {
        r.add("DFM-POSITION", "error",
              "ag_implement_3point_hitch_pin: pin_position '" + in.pin_position +
              "' invalid (must be top_link or lower_link)");
    }
    const double pinDia = pinDiameterForCategory(in.hitch_category);
    if (pinDia > 0.0 && in.linch_pin_hole_dia_mm >= pinDia / 2.0) {
        r.add("DFM-LINCH", "error",
              "ag_implement_3point_hitch_pin: linch_pin_hole_dia " +
              std::to_string(in.linch_pin_hole_dia_mm) +
              " mm must be < pin_dia/2 (" +
              std::to_string(pinDia / 2.0) + " mm)");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ag_implement_3point_hitch_pin DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double pinDia = pinDiameterForCategory(in.hitch_category);
    if (pinDia <= 0.0)
        throw SkillError("ag_implement_3point_hitch_pin: category lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double topZ = zMax;
    const double thickness = zMax - zMin;

    // ── 1) Primary hitch pin bore (Z-axis, full through-thickness) ──
    const gp_Pnt pinStart(cx, cy, zMin - kOver);
    const gp_Ax2 pinAx(pinStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(pinAx, pinDia / 2.0, thickness + 2.0 * kOver));

    // ── 2) Cross linch-pin retention hole (Y-axis, perpendicular) ──
    //
    // Linch pin sits across the top of the main hitch pin to retain it.
    // Drilled perpendicular to the main pin axis at z = (top_link uses
    // upper portion, lower_link uses center).  Cross hole runs the full
    // Y extent of the workpiece.
    const double linchZ = (in.pin_position == "top_link")
                          ? topZ - pinDia * 0.25
                          : (zMin + zMax) * 0.5;
    const gp_Pnt linchStart(cx, yMin - kOver, linchZ);
    const gp_Ax2 linchAx(linchStart, gp::DY());
    current = pr::cut(
        current,
        pr::cylinder(linchAx, in.linch_pin_hole_dia_mm / 2.0,
                     (yMax - yMin) + 2.0 * kOver));

    const double vPin = M_PI * std::pow(pinDia / 2.0, 2) * thickness;
    const double vLinch = M_PI * std::pow(in.linch_pin_hole_dia_mm / 2.0, 2)
                          * (yMax - yMin);
    const double volRemoved = vPin + vLinch;
    (void)xMin; (void)xMax;

    json params = {
        { "center_xy",             { in.center_xy.X(),
                                     in.center_xy.Y(),
                                     in.center_xy.Z() } },
        { "hitch_category",        in.hitch_category },
        { "pin_position",          in.pin_position },
        { "linch_pin_hole_dia_mm", in.linch_pin_hole_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "ag_feature_type",            "three_point_hitch_pin_mount" },
        { "subfeature_count",           2 },
        { "hitch_category",             in.hitch_category },
        { "pin_position",               in.pin_position },
        { "derived_pin_dia_mm",         pinDia },
        { "linch_pin_hole_dia_mm",      in.linch_pin_hole_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ASAE S217 / ISO 730" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;cross_drill";
    tooling.tool_dia_mm       = pinDia;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(30.0, 25.0);
    tooling.extra = {
        { "ag_application", "3point_hitch_pin_mount" },
        { "standard",       "ASAE S217" },
        { "category",       in.hitch_category },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::ag_implement_3point_hitch_pin: cat={} pos={} pinDia={}",
                  in.hitch_category, in.pin_position, pinDia);

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

    // Geometric: cylinder at category-typical radius (9.5 / 12.7 / 15.9 mm)
    // + perpendicular cross cylinder of smaller radius.
    int mainPin = 0;
    int crossPin = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 8.5 && radius <= 17.0) ++mainPin;
            else if (radius >= 2.0 && radius < 8.5) ++crossPin;
        } catch (...) {}
    }
    if (mainPin >= 1 && crossPin >= 1) {
        int catGuess = 1;
        json recovered = { { "hitch_category",        catGuess },
                           { "pin_position",          "top_link" },
                           { "linch_pin_hole_dia_mm", 6.0 } };
        json matched   = { { "source", "geometric_hitch_pin_pattern" },
                           { "main_pin_cyls", mainPin },
                           { "cross_pin_cyls", crossPin } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::ag_implement_3point_hitch_pin
