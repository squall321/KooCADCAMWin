// @lat: [[engine/skills#winch_drum_base_bolt_circle]]

#include "winch_drum_base_bolt_circle.hpp"

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

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::winch_drum_base_bolt_circle {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.base_bolt_circle_dia_mm <= 0.0 || in.bolt_dia_mm <= 0.0 ||
        in.shaft_bore_dia_mm <= 0.0 || in.pawl_pocket_len_mm <= 0.0 ||
        in.pawl_pocket_wid_mm <= 0.0 || in.pawl_pocket_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "winch_drum_base_bolt_circle: all dimensions must be > 0");
    }

    if (in.bolt_count < 3 || in.bolt_count > 8) {
        r.add("DFM-MARINE-COUNT", "error",
              "winch_drum_base_bolt_circle: bolt_count (" +
              std::to_string(in.bolt_count) +
              ") must be in [3, 8] for a marine winch base");
    }

    if (in.base_bolt_circle_dia_mm <= in.shaft_bore_dia_mm) {
        r.add("DFM-MARINE-SHAFT", "error",
              "winch_drum_base_bolt_circle: base_bolt_circle_dia_mm must "
              "exceed shaft_bore_dia_mm (bolts would intersect the shaft bore)");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "winch_drum_base_bolt_circle DFM failed:";
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

    // ── 1..N) Bolt holes on PCD via SEQUENTIAL SetRotation ─────────────
    const double pcdR  = in.base_bolt_circle_dia_mm / 2.0;
    const double boltR = in.bolt_dia_mm / 2.0;
    const gp_Pnt boltEntryTpl(cx + pcdR, cy, zMin - kOver);
    const gp_Ax2 boltAxTpl(boltEntryTpl, gp::DZ());
    const TopoDS_Shape boltTemplate = pr::cylinder(boltAxTpl, boltR, thru);

    const gp_Ax1 rotAxis(gp_Pnt(cx, cy, topZ), gp::DZ());
    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(boltTemplate, rot, true);
        if (!xform.IsDone())
            throw SkillError("winch_drum_base_bolt_circle: bolt rotation failed");
        current = pr::cut(current, xform.Shape());  // sequential — no compound
    }

    // ── N+1) Central shaft bore ────────────────────────────────────────
    const double shaftR = in.shaft_bore_dia_mm / 2.0;
    const gp_Ax2 shaftAx(gp_Pnt(cx, cy, zMin - kOver), gp::DZ());
    current = pr::cut(current, pr::cylinder(shaftAx, shaftR, thru));

    // ── N+2) Pawl pocket (rectangular box recess on top face) ──────────
    // Placed just outside the shaft bore in +X, recessed from the top face.
    const double pawlLen   = in.pawl_pocket_len_mm;
    const double pawlWid   = in.pawl_pocket_wid_mm;
    const double pawlDepth = in.pawl_pocket_depth_mm;
    const double pawlCx    = cx + shaftR + pawlLen / 2.0 + 2.0;
    const gp_Pnt pawlOrigin(pawlCx - pawlLen / 2.0,
                            cy - pawlWid / 2.0,
                            topZ - pawlDepth);
    const gp_Ax2 pawlAx(pawlOrigin, gp::DZ());
    current = pr::cut(current, pr::box(pawlAx, pawlLen, pawlWid, pawlDepth + kOver));

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double plateThk = (zMax - zMin);
    const double vBolts = static_cast<double>(in.bolt_count) *
                          M_PI * boltR * boltR * plateThk;
    const double vShaft = M_PI * shaftR * shaftR * plateThk;
    const double vPawl  = pawlLen * pawlWid * pawlDepth;
    const double volRemoved = vBolts + vShaft + vPawl;

    json bolts_json = json::array();
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        bolts_json.push_back({
            { "x_mm",      cx + pcdR * std::cos(theta) },
            { "y_mm",      cy + pcdR * std::sin(theta) },
            { "theta_rad", theta },
        });
    }

    const int subfeatureCount = in.bolt_count + 2;  // N bolts + shaft + pawl

    json params = {
        { "center_xy",                { in.center_xy.X(),
                                        in.center_xy.Y(),
                                        in.center_xy.Z() } },
        { "base_bolt_circle_dia_mm",  in.base_bolt_circle_dia_mm },
        { "bolt_count",               in.bolt_count },
        { "bolt_dia_mm",              in.bolt_dia_mm },
        { "shaft_bore_dia_mm",        in.shaft_bore_dia_mm },
        { "pawl_pocket_len_mm",       in.pawl_pocket_len_mm },
        { "pawl_pocket_wid_mm",       in.pawl_pocket_wid_mm },
        { "pawl_pocket_depth_mm",     in.pawl_pocket_depth_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "marine_feature_type",        "winch_drum_base" },
        { "subfeature_count",           subfeatureCount },
        { "base_bolt_circle_dia_mm",    in.base_bolt_circle_dia_mm },
        { "bolt_count",                 in.bolt_count },
        { "bolt_dia_mm",                in.bolt_dia_mm },
        { "shaft_bore_dia_mm",          in.shaft_bore_dia_mm },
        { "bolt_positions",             bolts_json },
        { "derived_pawl_len_mm",        pawlLen },
        { "derived_pawl_wid_mm",        pawlWid },
        { "derived_pawl_depth_mm",      pawlDepth },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "marine winch base bolt-circle practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;boring_bar;end_mill";
    tooling.tool_dia_mm       = std::max(in.shaft_bore_dia_mm, in.bolt_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 60.0 + 5.0 * static_cast<double>(in.bolt_count);
    tooling.extra = {
        { "marine_application", "winch_drum_base" },
        { "removed_volume_mm3", volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::winch_drum_base_bolt_circle: pcd={} bolts={} shaft={} faces {}→{}",
                  in.base_bolt_circle_dia_mm, in.bolt_count,
                  in.shaft_bore_dia_mm, wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: one central +Z shaft bore plus >= 3 bolt cylinders.
    int shaftCyls = 0;
    int boltCyls  = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 8.0) ++shaftCyls;
            else if (radius >= 1.5 && radius < 8.0) ++boltCyls;
        } catch (...) {}
    }
    if (shaftCyls >= 1 && boltCyls >= 3) {
        json recovered = { { "base_bolt_circle_dia_mm", 90.0 },
                           { "bolt_count",              static_cast<int>(boltCyls) } };
        json matched   = { { "source",      "geometric_winch_base" },
                           { "shaft_cyls",  shaftCyls },
                           { "bolt_cyls",   boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::winch_drum_base_bolt_circle
