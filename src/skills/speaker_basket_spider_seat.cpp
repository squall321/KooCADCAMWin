// @lat: [[engine/skills#speaker_basket_spider_seat]]

#include "speaker_basket_spider_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::speaker_basket_spider_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.spider_ledge_dia_mm > 0.0) || !(in.ledge_depth_mm > 0.0) ||
        !(in.vc_clearance_dia_mm > 0.0) || !(in.vent_dia_mm > 0.0) ||
        !(in.vent_circle_dia_mm > 0.0) || in.vent_count <= 0) {
        r.add("DFM-INPUT", "error",
              "speaker_basket_spider_seat: all dims and vent_count must be > 0");
        return r;
    }
    if (!(in.vc_clearance_dia_mm < in.spider_ledge_dia_mm)) {
        r.add("DFM-VC-BORE", "error",
              "speaker_basket_spider_seat: vc_clearance_dia (" +
              std::to_string(in.vc_clearance_dia_mm) +
              ") must be < spider_ledge_dia (" +
              std::to_string(in.spider_ledge_dia_mm) + ")");
    }
    // Vent circle must sit between the VC bore wall and the ledge OD.
    const double ventPcdR = in.vent_circle_dia_mm / 2.0;
    const double ventR    = in.vent_dia_mm / 2.0;
    if (ventPcdR - ventR <= in.vc_clearance_dia_mm / 2.0 ||
        ventPcdR + ventR >= in.spider_ledge_dia_mm / 2.0) {
        r.add("DFM-VENT-FIT", "error",
              "speaker_basket_spider_seat: vent circle does not fit between "
              "VC bore and ledge OD");
    }
    // Circumferential overlap: chord between vents must exceed vent_dia.
    const double chord = 2.0 * ventPcdR *
                         std::sin(M_PI / static_cast<double>(in.vent_count));
    if (chord < in.vent_dia_mm * 1.5) {
        r.add("DFM-VENT-FIT", "warning",
              "speaker_basket_spider_seat: vent chord spacing " +
              std::to_string(chord) + " mm tight for vent_dia");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "speaker_basket_spider_seat DFM failed:";
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

    // ── 1) Spider ledge counterbore (wide, shallow) ──────────────────────
    const double ledgeR = in.spider_ledge_dia_mm / 2.0;
    const gp_Pnt ledgeStart(cx, cy, topZ - in.ledge_depth_mm);
    const gp_Ax2 ledgeAx(ledgeStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(ledgeAx, ledgeR, in.ledge_depth_mm + kOver));

    // ── 2) Voice-coil clearance bore (narrow, through) ───────────────────
    const double vcR = in.vc_clearance_dia_mm / 2.0;
    const gp_Pnt vcStart(cx, cy, zMin - kOver);
    const gp_Ax2 vcAx(vcStart, gp::DZ());
    current = pr::cut(current, pr::cylinder(vcAx, vcR, thru));

    // ── 3) N vent holes around the ledge (SetRotation) ───────────────────
    const gp_Pnt center(cx, cy, topZ);
    const gp_Ax1 rotAxis(center, gp::DZ());
    const double ventR = in.vent_dia_mm / 2.0;
    const double pcdR  = in.vent_circle_dia_mm / 2.0;
    const gp_Pnt seedStart(cx + pcdR, cy, zMin - kOver);
    const gp_Ax2 seedAx(seedStart, gp::DZ());
    const TopoDS_Shape ventSeed = pr::cylinder(seedAx, ventR, thru);
    for (int i = 0; i < in.vent_count; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) /
                             static_cast<double>(in.vent_count);
        gp_Trsf t;
        t.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(ventSeed, t, true);
        if (!xform.IsDone())
            throw SkillError("speaker_basket_spider_seat: vent rotation failed");
        current = pr::cut(current, xform.Shape());
    }

    const int subfeatures = 2 + in.vent_count;
    const double vLedge = M_PI * ledgeR * ledgeR * in.ledge_depth_mm
                          - M_PI * vcR * vcR * in.ledge_depth_mm;
    const double vVc    = M_PI * vcR * vcR * (zMax - zMin);
    const double vVents = M_PI * ventR * ventR * (zMax - zMin) * in.vent_count;
    const double volRemoved = std::max(0.0, vLedge) + vVc + vVents;

    json params = {
        { "center_xy",           { in.center_xy.X(),
                                   in.center_xy.Y(),
                                   in.center_xy.Z() } },
        { "spider_ledge_dia_mm", in.spider_ledge_dia_mm },
        { "ledge_depth_mm",      in.ledge_depth_mm },
        { "vc_clearance_dia_mm", in.vc_clearance_dia_mm },
        { "vent_count",          in.vent_count },
        { "vent_dia_mm",         in.vent_dia_mm },
        { "vent_circle_dia_mm",  in.vent_circle_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "audio_feature_type",         "spider_seat_vented" },
        { "subfeature_count",           subfeatures },
        { "vent_count",                 in.vent_count },
        { "derived_ledge_dia_mm",       in.spider_ledge_dia_mm },
        { "derived_vc_bore_dia_mm",     in.vc_clearance_dia_mm },
        { "derived_vent_circle_dia_mm", in.vent_circle_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "counterbore;drill";
    tooling.tool_dia_mm       = in.spider_ledge_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(20.0, 6.0 + in.vent_count * 2.0);
    tooling.extra = {
        { "audio_application", "driver_basket_spider_seat" },
        { "vent_count",        in.vent_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::speaker_basket_spider_seat: ledge={} vc={} vents={}",
                  in.spider_ledge_dia_mm, in.vc_clearance_dia_mm, in.vent_count);

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

    // Geometric: a wide shallow counterbore + a narrow concentric bore +
    // several small vent cylinders.
    int wideCyls   = 0;
    int narrowCyls = 0;
    int ventCyls   = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 15.0) ++wideCyls;
            else if (radius >= 5.0 && radius < 15.0) ++narrowCyls;
            else if (radius < 5.0) ++ventCyls;
        } catch (...) {}
    }
    if (wideCyls >= 1 && narrowCyls >= 1 && ventCyls >= 2) {
        json recovered = { { "spider_ledge_dia_mm", 40.0 },
                           { "vc_clearance_dia_mm", 26.0 },
                           { "vent_count",          ventCyls } };
        json matched   = { { "source",      "geometric_spider_seat_pattern" },
                           { "wide_cyls",   wideCyls },
                           { "narrow_cyls", narrowCyls },
                           { "vent_cyls",   ventCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::speaker_basket_spider_seat
