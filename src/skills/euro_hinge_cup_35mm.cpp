// @lat: [[engine/skills#euro_hinge_cup_35mm]]

#include "euro_hinge_cup_35mm.hpp"

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

namespace koocadcam::skill::euro_hinge_cup_35mm {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.cup_dia_mm > 0.0) || !(in.cup_depth_mm > 0.0) ||
        !(in.screw_spacing_mm > 0.0) || !(in.screw_pilot_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "euro_hinge_cup_35mm: all dimensions must be > 0");
        return r;
    }
    if (in.cup_dia_mm < 25.0 || in.cup_dia_mm > 40.0) {
        r.add("DFM-CUPDIA", "error",
              "euro_hinge_cup_35mm: cup_dia_mm " +
              std::to_string(in.cup_dia_mm) +
              " out of range [25, 40] for a concealed-hinge cup");
    }
    // Pilot holes must straddle the cup wall, not pierce it.
    const double pilotInner = in.screw_spacing_mm / 2.0 - in.screw_pilot_dia_mm / 2.0;
    if (pilotInner <= in.cup_dia_mm / 2.0) {
        r.add("DFM-PILOT", "error",
              "euro_hinge_cup_35mm: screw_spacing too small — pilot holes "
              "would break into the cup bore");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "euro_hinge_cup_35mm DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Flat-bottom cup bore (blind) ──────────────────────────────────
    const double cupR = in.cup_dia_mm / 2.0;
    const gp_Pnt cupStart(cx, cy, topZ - in.cup_depth_mm);
    const gp_Ax2 cupAx(cupStart, gp::DZ());
    TopoDS_Shape current =
        pr::cut(wp.shape(), pr::cylinder(cupAx, cupR, in.cup_depth_mm + kOver));

    // ── 2..3) Two screw pilot holes straddling the cup along X ───────────
    const double pilotR = in.screw_pilot_dia_mm / 2.0;
    const double pilotDepth = std::min(in.cup_depth_mm + 3.0, (zMax - zMin));
    const double xOff = in.screw_spacing_mm / 2.0;
    for (int i = 0; i < 2; ++i) {
        const double px = (i == 0) ? cx - xOff : cx + xOff;
        const gp_Pnt pStart(px, cy, topZ - pilotDepth);
        const gp_Ax2 pAx(pStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(pAx, pilotR, pilotDepth + kOver));
    }

    const int subfeatures = 1 + 2;
    const double vCup    = M_PI * cupR * cupR * in.cup_depth_mm;
    const double vPilots = M_PI * pilotR * pilotR * pilotDepth * 2.0;
    const double volRemoved = vCup + vPilots;

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "cup_dia_mm",         in.cup_dia_mm },
        { "cup_depth_mm",       in.cup_depth_mm },
        { "screw_spacing_mm",   in.screw_spacing_mm },
        { "screw_pilot_dia_mm", in.screw_pilot_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "furniture_feature_type",     "euro_concealed_hinge_cup" },
        { "subfeature_count",           subfeatures },
        { "pilot_count",                2 },
        { "derived_cup_dia_mm",         in.cup_dia_mm },
        { "derived_cup_depth_mm",       in.cup_depth_mm },
        { "derived_screw_spacing_mm",   in.screw_spacing_mm },
        { "derived_pilot_dia_mm",       in.screw_pilot_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "Euro 35mm system" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "forstner;drill";
    tooling.tool_dia_mm       = in.cup_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 150.0;
    tooling.feed_per_tooth_mm = 0.08;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(15.0, in.cup_depth_mm * 1.5 + 8.0);
    tooling.extra = {
        { "furniture_application", "concealed_hinge_cup" },
        { "standard",              "Euro 35mm system" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::euro_hinge_cup_35mm: cup_dia={} depth={} spacing={}",
                  in.cup_dia_mm, in.cup_depth_mm, in.screw_spacing_mm);

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

    // Geometric: one large cup cylinder (~17.5 mm radius) plus two small
    // pilot cylinders.
    int cupCyls = 0;
    int pilotCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 12.0 && radius <= 20.0) ++cupCyls;
            else if (radius >= 0.5 && radius <= 4.0) ++pilotCyls;
        } catch (...) {}
    }
    if (cupCyls >= 1 && pilotCyls >= 2) {
        json recovered = { { "cup_dia_mm",         35.0 },
                           { "cup_depth_mm",       11.5 },
                           { "screw_spacing_mm",   45.0 },
                           { "screw_pilot_dia_mm", 2.5 } };
        json matched   = { { "source",     "geometric_cup_pilot_pattern" },
                           { "cup_cyls",   cupCyls },
                           { "pilot_cyls", pilotCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::euro_hinge_cup_35mm
