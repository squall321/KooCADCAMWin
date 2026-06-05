// @lat: [[engine/skills#xlr_panel_d_cutout]]

#include "xlr_panel_d_cutout.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::xlr_panel_d_cutout {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
constexpr double kBoreMin = 20.0;
constexpr double kBoreMax = 30.0;

// Vertical drop placing the mount-hole centre line below the bore centre so
// the holes clear the bore wall while keeping the X spacing as specified.
double mountYDrop(double boreR, double mountR, double spacing)
{
    const double need = boreR + mountR + 1.0;        // required radial clearance
    const double xOff = spacing / 2.0;
    const double y2   = need * need - xOff * xOff;
    return (y2 > 0.0) ? std::sqrt(y2) : need;
}
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.mount_hole_dia_mm > 0.0) || !(in.mount_spacing_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "xlr_panel_d_cutout: mount hole dia and spacing must be > 0");
        return r;
    }
    if (!(in.bore_dia_mm >= kBoreMin && in.bore_dia_mm <= kBoreMax)) {
        r.add("DFM-BORE", "error",
              "xlr_panel_d_cutout: bore_dia (" + std::to_string(in.bore_dia_mm) +
              ") outside Neutrik D range [20, 30] mm");
    }
    // Mount holes must clear the bore: spacing/2 (X) alone may sit inside the
    // bore; the skill drops them in Y, but reject impossible spacing.
    const double boreR  = in.bore_dia_mm / 2.0;
    const double mountR = in.mount_hole_dia_mm / 2.0;
    if (in.mount_spacing_mm / 2.0 + mountR >= boreR + 30.0) {
        r.add("DFM-MOUNT", "error",
              "xlr_panel_d_cutout: mount spacing too wide for panel footprint");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "xlr_panel_d_cutout DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;

    const double boreR  = in.bore_dia_mm / 2.0;
    const double mountR = in.mount_hole_dia_mm / 2.0;

    // ── 1) Main connector bore (through) ─────────────────────────────────
    const gp_Pnt boreStart(cx, cy, zMin - kOver);
    const gp_Ax2 boreAx(boreStart, gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(), pr::cylinder(boreAx, boreR, thru));

    // ── 2..3) Two mounting holes at +/- spacing/2 in X, dropped in Y ─────
    const double yDrop = mountYDrop(boreR, mountR, in.mount_spacing_mm);
    const double mountY = cy - yDrop;
    const double xOff = in.mount_spacing_mm / 2.0;
    for (int i = 0; i < 2; ++i) {
        const double mx = (i == 0) ? cx - xOff : cx + xOff;
        const gp_Pnt mStart(mx, mountY, zMin - kOver);
        const gp_Ax2 mAx(mStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(mAx, mountR, thru));
    }

    const double vBore  = M_PI * boreR * boreR * (zMax - zMin);
    const double vMount = M_PI * mountR * mountR * (zMax - zMin) * 2.0;
    const double volRemoved = vBore + vMount;

    json params = {
        { "center_xy",        { in.center_xy.X(),
                                in.center_xy.Y(),
                                in.center_xy.Z() } },
        { "bore_dia_mm",      in.bore_dia_mm },
        { "mount_hole_dia_mm",in.mount_hole_dia_mm },
        { "mount_spacing_mm", in.mount_spacing_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "audio_feature_type",         "neutrik_d_panel_cutout" },
        { "subfeature_count",           3 },
        { "derived_bore_dia_mm",        in.bore_dia_mm },
        { "derived_mount_hole_dia_mm",  in.mount_hole_dia_mm },
        { "derived_mount_spacing_mm",   in.mount_spacing_mm },
        { "derived_mount_y_drop_mm",    yDrop },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "Neutrik D-series" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "punch;drill";
    tooling.tool_dia_mm       = in.bore_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 240.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(10.0, 6.0);
    tooling.extra = {
        { "audio_application", "xlr_panel_connector" },
        { "standard",          "Neutrik D-series" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::xlr_panel_d_cutout: bore={} mount={} spacing={}",
                  in.bore_dia_mm, in.mount_hole_dia_mm, in.mount_spacing_mm);

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

    // Geometric: one ~24 mm bore plus two small (~3 mm) mounting holes.
    int boreCyls  = 0;
    int mountCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 10.0 && radius <= 15.0) ++boreCyls;
            else if (radius >= 1.0 && radius <= 3.0) ++mountCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1 && mountCyls >= 2) {
        json recovered = { { "bore_dia_mm",       24.0 },
                           { "mount_hole_dia_mm", 3.2 },
                           { "mount_spacing_mm",  19.05 } };
        json matched   = { { "source",     "geometric_xlr_d_pattern" },
                           { "bore_cyls",  boreCyls },
                           { "mount_cyls", mountCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::xlr_panel_d_cutout
