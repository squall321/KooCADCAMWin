// @lat: [[engine/skills#harmonic_drive_flange_bore]]

#include "harmonic_drive_flange_bore.hpp"

#include "Workpiece.hpp"
#include "_iso286_fits.hpp"
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

namespace koocadcam::skill::harmonic_drive_flange_bore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.flange_bore_dia_mm <= 0.0 || in.bolt_circle_dia_mm <= 0.0 ||
        in.bolt_dia_mm <= 0.0 || in.dowel_dia_mm <= 0.0 ||
        in.dowel_circle_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "harmonic_drive_flange_bore: all dimensions must be > 0");
    }

    if (in.bolt_count < 3 || in.bolt_count > 12) {
        r.add("DFM-ROBOTICS-COUNT", "error",
              "harmonic_drive_flange_bore: bolt_count (" +
              std::to_string(in.bolt_count) +
              ") must be in [3, 12] for a strain-wave output flange");
    }

    if (in.bolt_circle_dia_mm <= in.flange_bore_dia_mm) {
        r.add("DFM-ROBOTICS-PCD", "error",
              "harmonic_drive_flange_bore: bolt_circle_dia_mm must exceed "
              "flange_bore_dia_mm (bolts would intersect the center bore)");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "harmonic_drive_flange_bore DFM failed:";
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

    // ── 1) Center bore (H7) — biggest feature first ────────────────────
    const double boreH7Max = iso286::h7_max_mm(in.flange_bore_dia_mm);
    const double boreR = boreH7Max / 2.0;
    const gp_Ax2 boreAx(gp_Pnt(cx, cy, zMin - kOver), gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(), pr::cylinder(boreAx, boreR, thru));

    // ── 2..N+1) Bolt circle via SEQUENTIAL SetRotation ─────────────────
    const double pcdR  = in.bolt_circle_dia_mm / 2.0;
    const double boltR = in.bolt_dia_mm / 2.0;
    const gp_Pnt boltEntryTpl(cx + pcdR, cy, zMin - kOver);
    const gp_Ax2 boltAxTpl(boltEntryTpl, gp::DZ());
    const TopoDS_Shape boltTemplate = pr::cylinder(boltAxTpl, boltR, thru);

    const gp_Ax1 rotAxis(gp_Pnt(cx, cy, topZ), gp::DZ());
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(boltTemplate, rot, true);
        if (!xform.IsDone())
            throw SkillError("harmonic_drive_flange_bore: bolt rotation failed");
        current = pr::cut(current, xform.Shape());  // sequential — no compound
    }

    // ── N+2..N+3) 2 locating dowel holes (0 deg and 180 deg) ───────────
    const double dowelPcdR = in.dowel_circle_dia_mm / 2.0;
    const double dowelR    = in.dowel_dia_mm / 2.0;
    const gp_Pnt dowelEntryTpl(cx + dowelPcdR, cy, zMin - kOver);
    const gp_Ax2 dowelAxTpl(dowelEntryTpl, gp::DZ());
    const TopoDS_Shape dowelTemplate = pr::cylinder(dowelAxTpl, dowelR, thru);
    for (int i = 0; i < 2; ++i) {
        const double theta = M_PI * static_cast<double>(i);  // 0, pi
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(dowelTemplate, rot, true);
        if (!xform.IsDone())
            throw SkillError("harmonic_drive_flange_bore: dowel rotation failed");
        current = pr::cut(current, xform.Shape());  // sequential — no compound
    }

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double plateThk = (zMax - zMin);
    const double vBore  = M_PI * boreR * boreR * plateThk;
    const double vBolts = static_cast<double>(in.bolt_count) *
                          M_PI * boltR * boltR * plateThk;
    const double vDowels = 2.0 * M_PI * dowelR * dowelR * plateThk;
    const double volRemoved = vBore + vBolts + vDowels;

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

    const int subfeatureCount = 1 + in.bolt_count + 2;

    json params = {
        { "center_xy",            { in.center_xy.X(),
                                    in.center_xy.Y(),
                                    in.center_xy.Z() } },
        { "flange_bore_dia_mm",   in.flange_bore_dia_mm },
        { "bolt_circle_dia_mm",   in.bolt_circle_dia_mm },
        { "bolt_count",           in.bolt_count },
        { "bolt_dia_mm",          in.bolt_dia_mm },
        { "dowel_dia_mm",         in.dowel_dia_mm },
        { "dowel_circle_dia_mm",  in.dowel_circle_dia_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "robotics_feature_type",      "harmonic_drive_output_flange" },
        { "subfeature_count",           subfeatureCount },
        { "flange_bore_dia_mm",         in.flange_bore_dia_mm },
        { "bolt_circle_dia_mm",         in.bolt_circle_dia_mm },
        { "bolt_count",                 in.bolt_count },
        { "bolt_dia_mm",                in.bolt_dia_mm },
        { "bolt_positions",             bolts_json },
        { "derived_bore_h7_max_mm",     boreH7Max },
        { "derived_dowel_circle_dia_mm", in.dowel_circle_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 286 H7 locating bore + bolt circle" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;drill";
    tooling.tool_dia_mm       = std::max(in.flange_bore_dia_mm, in.bolt_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 240.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 50.0 + 5.0 * static_cast<double>(in.bolt_count) + 20.0;
    tooling.extra = {
        { "robotics_application", "strain_wave_output_flange" },
        { "removed_volume_mm3",   volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::harmonic_drive_flange_bore: bore={} pcd={} bolts={} faces {}→{}",
                  in.flange_bore_dia_mm, in.bolt_circle_dia_mm,
                  in.bolt_count, wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: one large +Z center bore plus >= 3 bolt cylinders.
    int boreCyls = 0;
    int boltCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 12.0) ++boreCyls;
            else if (radius >= 1.5 && radius < 12.0) ++boltCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1 && boltCyls >= 3) {
        json recovered = { { "flange_bore_dia_mm", 40.0 },
                           { "bolt_count",         static_cast<int>(boltCyls) } };
        json matched   = { { "source",    "geometric_harmonic_flange" },
                           { "bore_cyls", boreCyls },
                           { "bolt_cyls", boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::harmonic_drive_flange_bore
