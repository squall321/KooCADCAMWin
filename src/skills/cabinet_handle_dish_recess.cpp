// @lat: [[engine/skills#cabinet_handle_dish_recess]]

#include "cabinet_handle_dish_recess.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::cabinet_handle_dish_recess {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// Bolt clearance diameter from the central tables; < 0 if key unknown.
double boltClearanceDia(const std::string& key)
{
    if (const tt::MetricThreadSpec* m = tt::findMetric(key))
        return m->clearance_medium_mm;
    if (const tt::UncUnfSpec* u = tt::findUncUnf(key))
        return u->clearance_normal_mm;
    return -1.0;
}
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.dish_length_mm > 0.0) || !(in.dish_width_mm > 0.0) ||
        !(in.dish_depth_mm > 0.0) || !(in.bolt_spacing_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "cabinet_handle_dish_recess: dish dims and bolt spacing must be > 0");
        return r;
    }
    if (in.bolt_thread_key.empty() ||
        boltClearanceDia(in.bolt_thread_key) < 0.0) {
        r.add("DFM-THREAD", "error",
              "cabinet_handle_dish_recess: bolt_thread_key '" +
              in.bolt_thread_key +
              "' not in central metric or UNC/UNF table");
    }
    // Both bolt holes must sit inside the dish footprint along X.
    const double boltDia = std::max(0.0, boltClearanceDia(in.bolt_thread_key));
    if (in.bolt_spacing_mm + boltDia >= in.dish_length_mm) {
        r.add("DFM-BOLTFIT", "error",
              "cabinet_handle_dish_recess: bolt spacing + bolt dia exceeds "
              "dish_length — bolts fall outside the dish");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cabinet_handle_dish_recess DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;

    const double boltDia = boltClearanceDia(in.bolt_thread_key);
    const double boltR   = boltDia / 2.0;

    // ── 1) Recessed dish pocket (blind, rounded-rect) ────────────────────
    const double cornerR = std::min(in.dish_width_mm, in.dish_length_mm) * 0.15;
    const gp_Pnt dishBottom(cx, cy, topZ - in.dish_depth_mm);
    const TopoDS_Shape dishTool = pr::roundedRectPocketTool(
        dishBottom, in.dish_length_mm, in.dish_width_mm,
        in.dish_depth_mm + kOver, cornerR);
    TopoDS_Shape current = pr::cut(wp.shape(), dishTool);

    // ── 2..3) Two bolt holes through the dish floor along X ──────────────
    const double xOff = in.bolt_spacing_mm / 2.0;
    for (int i = 0; i < 2; ++i) {
        const double bx = (i == 0) ? cx - xOff : cx + xOff;
        const gp_Pnt bStart(bx, cy, zMin - kOver);
        const gp_Ax2 bAx(bStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(bAx, boltR, thru));
    }

    const int subfeatures = 1 + 2;
    const double vDish  = in.dish_length_mm * in.dish_width_mm * in.dish_depth_mm;
    const double vBolts = M_PI * boltR * boltR * (zMax - zMin) * 2.0;
    const double volRemoved = vDish + vBolts;

    json params = {
        { "center_xy",       { in.center_xy.X(),
                               in.center_xy.Y(),
                               in.center_xy.Z() } },
        { "dish_length_mm",  in.dish_length_mm },
        { "dish_width_mm",   in.dish_width_mm },
        { "dish_depth_mm",   in.dish_depth_mm },
        { "bolt_spacing_mm", in.bolt_spacing_mm },
        { "bolt_thread_key", in.bolt_thread_key },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "audio_feature_type",         "recessed_bar_handle_dish" },
        { "subfeature_count",           subfeatures },
        { "derived_bolt_clearance_dia_mm", boltDia },
        { "derived_dish_length_mm",     in.dish_length_mm },
        { "derived_dish_depth_mm",      in.dish_depth_mm },
        { "derived_bolt_spacing_mm",    in.bolt_spacing_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::max(6.0, boltDia);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 230.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(20.0, in.dish_length_mm / 4.0 + 6.0);
    tooling.extra = {
        { "audio_application", "cabinet_recessed_handle" },
        { "bolt_thread_key",   in.bolt_thread_key },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cabinet_handle_dish_recess: dish={}x{}x{} bolts='{}'",
                  in.dish_length_mm, in.dish_width_mm, in.dish_depth_mm,
                  in.bolt_thread_key);

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

    // Geometric: a rounded-rect pocket (several planar walls + rounded
    // corners) plus two small bolt cylinders through the floor.
    int boltCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 2.0 && radius <= 8.0) ++boltCyls;
        } catch (...) {}
    }
    if (boltCyls >= 2) {
        json recovered = { { "dish_length_mm",  120.0 },
                           { "dish_width_mm",   45.0 },
                           { "dish_depth_mm",   18.0 },
                           { "bolt_spacing_mm", 90.0 },
                           { "bolt_thread_key", "M6" } };
        json matched   = { { "source",    "geometric_dish_bolt_pattern" },
                           { "bolt_cyls", boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::cabinet_handle_dish_recess
