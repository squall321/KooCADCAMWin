// @lat: [[engine/skills#brake_disc_hub_bolt_circle]]

#include "brake_disc_hub_bolt_circle.hpp"

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

namespace koocadcam::skill::brake_disc_hub_bolt_circle {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hub_bore_dia_mm <= 0.0 || in.bolt_circle_dia_mm <= 0.0 ||
        in.bolt_dia_mm <= 0.0 || in.vent_slot_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "brake_disc_hub_bolt_circle: all dimensions must be > 0");
    }

    if (in.bolt_count < 4 || in.bolt_count > 12) {
        r.add("DFM-COUNT", "error",
              "brake_disc_hub_bolt_circle: bolt_count (" +
              std::to_string(in.bolt_count) + ") must be in [4, 12]");
    }

    if (in.bolt_circle_dia_mm <= in.hub_bore_dia_mm) {
        r.add("DFM-PCD", "error",
              "brake_disc_hub_bolt_circle: bolt_circle_dia_mm must exceed "
              "hub_bore_dia_mm (bolts would intersect the hub bore)");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "brake_disc_hub_bolt_circle DFM failed:";
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

    const gp_Ax1 rotAxis(gp_Pnt(cx, cy, topZ), gp::DZ());

    // ── 1) Central hub bore (through) ─────────────────────────────────
    const double hubR = in.hub_bore_dia_mm / 2.0;
    const gp_Ax2 hubAx(gp_Pnt(cx, cy, zMin - kOver), gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(),
                                   pr::cylinder(hubAx, hubR, thru));

    // ── 2..N) Bolt holes on the PCD via SEQUENTIAL SetRotation ────────
    const double pcdR  = in.bolt_circle_dia_mm / 2.0;
    const double boltR = in.bolt_dia_mm / 2.0;
    const gp_Pnt boltEntryTpl(cx + pcdR, cy, zMin - kOver);
    const gp_Ax2 boltAxTpl(boltEntryTpl, gp::DZ());
    const TopoDS_Shape boltTemplate = pr::cylinder(boltAxTpl, boltR, thru);
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(boltTemplate, rot, true);
        if (!xform.IsDone())
            throw SkillError("brake_disc_hub_bolt_circle: bolt rotation failed");
        current = pr::cut(current, xform.Shape());   // sequential — no compound
    }

    // ── (N+1)..end) Radial ventilation slots via SEQUENTIAL SetRotation
    // Each slot is a radial box just outside the PCD, running outward in +X
    // (length), with width in Y.  Cut as a through slot, indexed by rotation.
    const double slotW   = in.vent_slot_width_mm;
    const double slotLen = std::max(8.0, pcdR * 0.6);          // radial length
    const double slotR0  = pcdR + boltR + 4.0;                  // inner radius
    const gp_Pnt slotOrigin(cx + slotR0, cy - slotW / 2.0, zMin - kOver);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ());
    const TopoDS_Shape slotTemplate = pr::box(slotAx, slotLen, slotW, thru);
    for (int i = 0; i < in.vent_slot_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.vent_slot_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(slotTemplate, rot, true);
        if (!xform.IsDone())
            throw SkillError("brake_disc_hub_bolt_circle: vent slot rotation failed");
        current = pr::cut(current, xform.Shape());   // sequential — no compound
    }

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ─────────────────────────────────────
    const double plateThk = (zMax - zMin);
    const double vHub   = M_PI * hubR * hubR * plateThk;
    const double vBolts = static_cast<double>(in.bolt_count) *
                          M_PI * boltR * boltR * plateThk;
    const double vVents = static_cast<double>(in.vent_slot_count) *
                          slotLen * slotW * plateThk;
    const double volRemoved = vHub + vBolts + vVents;

    const int subfeatureCount = 1 + in.bolt_count + in.vent_slot_count;

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

    json params = {
        { "center_xy",            { in.center_xy.X(),
                                    in.center_xy.Y(),
                                    in.center_xy.Z() } },
        { "hub_bore_dia_mm",      in.hub_bore_dia_mm },
        { "bolt_circle_dia_mm",   in.bolt_circle_dia_mm },
        { "bolt_count",           in.bolt_count },
        { "bolt_dia_mm",          in.bolt_dia_mm },
        { "vent_slot_count",      in.vent_slot_count },
        { "vent_slot_width_mm",   in.vent_slot_width_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "railway_feature_type",       "brake_disc_hub_bolt_circle" },
        { "subfeature_count",           subfeatureCount },
        { "hub_bore_dia_mm",            in.hub_bore_dia_mm },
        { "bolt_circle_dia_mm",         in.bolt_circle_dia_mm },
        { "bolt_count",                 in.bolt_count },
        { "bolt_dia_mm",                in.bolt_dia_mm },
        { "vent_slot_count",            in.vent_slot_count },
        { "bolt_positions",             bolts_json },
        { "derived_vent_slot_len_mm",   slotLen },
        { "derived_vent_slot_width_mm", slotW },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "railway brake-disc hub bolt-circle practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill";
    tooling.tool_dia_mm       = std::max(in.bolt_dia_mm, in.hub_bore_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 60.0 + 4.0 * static_cast<double>(in.bolt_count)
                                + 6.0 * static_cast<double>(in.vent_slot_count);
    tooling.extra = {
        { "railway_application", "brake_disc_hub_bolt_circle" },
        { "removed_volume_mm3",  volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::brake_disc_hub_bolt_circle: bore={} pcd={} bolts={} vents={} faces {}→{}",
                  in.hub_bore_dia_mm, in.bolt_circle_dia_mm, in.bolt_count,
                  in.vent_slot_count, wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: one central +Z hub bore plus >= 4 bolt cylinders.
    int hubCyls  = 0;
    int boltCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 20.0) ++hubCyls;
            else if (radius >= 2.0) ++boltCyls;
        } catch (...) {}
    }
    if (hubCyls >= 1 && boltCyls >= 4) {
        json recovered = { { "hub_bore_dia_mm",    80.0 },
                           { "bolt_circle_dia_mm", 160.0 },
                           { "bolt_count",         static_cast<int>(boltCyls) },
                           { "vent_slot_count",    8 } };
        json matched   = { { "source",    "geometric_brake_disc_hub" },
                           { "hub_cyls",  hubCyls },
                           { "bolt_cyls", boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::brake_disc_hub_bolt_circle
