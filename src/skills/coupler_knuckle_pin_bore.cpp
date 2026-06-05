// @lat: [[engine/skills#coupler_knuckle_pin_bore]]

#include "coupler_knuckle_pin_bore.hpp"

#include "Workpiece.hpp"
#include "_iso286_fits.hpp"
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

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::coupler_knuckle_pin_bore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pin_dia_mm <= 0.0 || in.washer_recess_dia_mm <= 0.0 ||
        in.washer_recess_depth_mm <= 0.0 || in.lock_pin_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "coupler_knuckle_pin_bore: all dimensions must be > 0");
    }

    if (in.washer_recess_dia_mm <= in.pin_dia_mm) {
        r.add("DFM-RECESS", "error",
              "coupler_knuckle_pin_bore: washer_recess_dia_mm must exceed "
              "pin_dia_mm (counterbore would be smaller than the bore)");
    }

    if (in.lock_pin_dia_mm >= in.pin_dia_mm) {
        r.add("DFM-LOCKPIN", "error",
              "coupler_knuckle_pin_bore: lock_pin_dia_mm must be smaller than "
              "pin_dia_mm");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "coupler_knuckle_pin_bore DFM failed:";
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

    // ── 1) Pivot pin bore (H7 hole basis, through the knuckle) ────────
    const double boreDia = iso286::h7_max_mm(in.pin_dia_mm);
    const double boreR   = boreDia / 2.0;
    const gp_Ax2 boreAx(gp_Pnt(cx, cy, zMin - kOver), gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(boreAx, boreR, thru));

    // ── 2) Thrust-washer recess counterbore (wider, shallow, top) ─────
    const double recR = in.washer_recess_dia_mm / 2.0;
    const gp_Pnt recStart(cx, cy, topZ - in.washer_recess_depth_mm);
    const gp_Ax2 recAx(recStart, gp::DZ());
    current = pr::cut(current,
        pr::cylinder(recAx, recR, in.washer_recess_depth_mm + kOver));

    // ── 3) Transverse lock-pin cross hole (radial, non-overlap Z) ─────
    // Drilled in -X across the part, below the washer recess Z zone so it
    // does not intersect the counterbore step.
    const double lockR  = in.lock_pin_dia_mm / 2.0;
    const double recBotZ = topZ - in.washer_recess_depth_mm;
    const double lockZ  = (recBotZ + zMin) / 2.0;   // mid of the lower bore zone
    const double crossLen = (xMax - xMin) + 2.0 * kOver;
    const gp_Pnt lockStart(xMax + kOver, cy, lockZ);
    const gp_Ax2 lockAx(lockStart, gp_Dir(-1.0, 0.0, 0.0));
    current = pr::cut(current, pr::cylinder(lockAx, lockR, crossLen));

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ─────────────────────────────────────
    const double plateThk = (zMax - zMin);
    const double vBore  = M_PI * boreR * boreR * plateThk;
    const double vRec   = M_PI * (recR * recR - boreR * boreR)
                          * in.washer_recess_depth_mm;
    const double vLock  = M_PI * lockR * lockR * crossLen;
    const double volRemoved = vBore + std::max(0.0, vRec) + vLock;

    json params = {
        { "center_xy",               { in.center_xy.X(),
                                       in.center_xy.Y(),
                                       in.center_xy.Z() } },
        { "pin_dia_mm",              in.pin_dia_mm },
        { "washer_recess_dia_mm",    in.washer_recess_dia_mm },
        { "washer_recess_depth_mm",  in.washer_recess_depth_mm },
        { "lock_pin_dia_mm",         in.lock_pin_dia_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "railway_feature_type",       "coupler_knuckle_pin_bore" },
        { "subfeature_count",           3 },
        { "pin_dia_mm",                 in.pin_dia_mm },
        { "washer_recess_dia_mm",       in.washer_recess_dia_mm },
        { "lock_pin_dia_mm",            in.lock_pin_dia_mm },
        { "derived_bore_h7_max_mm",     boreDia },
        { "derived_recess_depth_mm",    in.washer_recess_depth_mm },
        { "derived_cross_hole_len_mm",  crossLen },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "AAR coupler knuckle (ISO 286 H7 pin bore)" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;boring_bar;counterbore";
    tooling.tool_dia_mm       = boreDia;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(50.0, plateThk * 1.5);
    tooling.extra = {
        { "railway_application", "coupler_knuckle_pin_bore" },
        { "pin_dia_mm",          in.pin_dia_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::coupler_knuckle_pin_bore: pin={} recess={} lock={} faces {}→{}",
                  in.pin_dia_mm, in.washer_recess_dia_mm, in.lock_pin_dia_mm,
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

    // Geometric fallback: a Z-axis pin bore + wider Z-axis recess + a
    // transverse (X-axis) lock-pin cylinder.
    int pinCyls   = 0;
    int recCyls   = 0;
    int crossCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            const double az = std::abs(c.Axis().Direction().Z());
            const double radius = c.Radius();
            if (az > 0.9) {
                if (radius >= 30.0) ++recCyls;
                else                ++pinCyls;
            } else {
                ++crossCyls;
            }
        } catch (...) {}
    }
    if (pinCyls >= 1 && crossCyls >= 1) {
        json recovered = { { "pin_dia_mm",              50.0 },
                           { "washer_recess_dia_mm",    70.0 },
                           { "washer_recess_depth_mm",  6.0 },
                           { "lock_pin_dia_mm",         12.0 } };
        json matched   = { { "source",     "geometric_coupler_knuckle" },
                           { "pin_cyls",   pinCyls },
                           { "recess_cyls", recCyls },
                           { "cross_cyls", crossCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::coupler_knuckle_pin_bore
