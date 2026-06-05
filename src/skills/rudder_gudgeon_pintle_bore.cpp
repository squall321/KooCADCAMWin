// @lat: [[engine/skills#rudder_gudgeon_pintle_bore]]

#include "rudder_gudgeon_pintle_bore.hpp"

#include "Workpiece.hpp"
#include "_iso286_fits.hpp"
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

namespace koocadcam::skill::rudder_gudgeon_pintle_bore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pintle_dia_mm <= 0.0 || in.bushing_od_mm <= 0.0 ||
        in.groove_width_mm <= 0.0 || in.bore_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "rudder_gudgeon_pintle_bore: all dimensions must be > 0");
    }

    if (in.bushing_od_mm <= in.pintle_dia_mm) {
        r.add("DFM-MARINE-FIT", "error",
              "rudder_gudgeon_pintle_bore: bushing_od_mm (" +
              std::to_string(in.bushing_od_mm) +
              ") must exceed pintle_dia_mm (" +
              std::to_string(in.pintle_dia_mm) +
              ") — bushing has no wall thickness");
    }

    if (in.groove_width_mm >= in.bore_depth_mm) {
        r.add("DFM-MARINE-GROOVE", "error",
              "rudder_gudgeon_pintle_bore: groove_width_mm (" +
              std::to_string(in.groove_width_mm) +
              ") must be < bore_depth_mm (" +
              std::to_string(in.bore_depth_mm) + ")");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rudder_gudgeon_pintle_bore DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Pintle bushing bore — H7 hole sized to the bushing OD ───────
    // For a press fit the bore (hole) is held at the H7 upper limit so the
    // interference with the bushing OD is controlled.
    const double boreFinishedDia = iso286::h7_max_mm(in.bushing_od_mm);
    const double boreR = boreFinishedDia / 2.0;
    const double boreDepth = in.bore_depth_mm;
    const gp_Ax2 boreAx(gp_Pnt(cx, cy, topZ - boreDepth), gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(),
                                   pr::cylinder(boreAx, boreR, boreDepth + kOver));

    // ── 2) Internal grease groove (annular ring) part-way down the bore ─
    const double grooveW   = in.groove_width_mm;
    const double grooveOD  = boreFinishedDia + 2.0;        // ~1 mm radial relief
    const double grooveID  = boreFinishedDia;              // starts at bore wall
    const double grooveZ   = topZ - (boreDepth * 0.5) - grooveW / 2.0;
    const gp_Ax2 grooveAx(gp_Pnt(cx, cy, grooveZ), gp::DZ());
    current = pr::cut(current,
                      pr::annularRing(grooveAx, grooveOD / 2.0,
                                      grooveID / 2.0, grooveW));

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double vBore   = M_PI * boreR * boreR * boreDepth;
    const double vGroove = M_PI *
        ((grooveOD * 0.5) * (grooveOD * 0.5) -
         (grooveID * 0.5) * (grooveID * 0.5)) * grooveW;
    const double volRemoved = vBore + vGroove;

    json params = {
        { "center_xy",       { in.center_xy.X(),
                               in.center_xy.Y(),
                               in.center_xy.Z() } },
        { "pintle_dia_mm",   in.pintle_dia_mm },
        { "bushing_od_mm",   in.bushing_od_mm },
        { "groove_width_mm", in.groove_width_mm },
        { "bore_depth_mm",   in.bore_depth_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "marine_feature_type",        "rudder_gudgeon_pintle_bore" },
        { "subfeature_count",           2 },
        { "pintle_dia_mm",              in.pintle_dia_mm },
        { "bushing_od_mm",              in.bushing_od_mm },
        { "derived_bore_h7_max_mm",     boreFinishedDia },
        { "derived_groove_id_mm",       grooveID },
        { "derived_groove_od_mm",       grooveOD },
        { "derived_groove_width_mm",    grooveW },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 286 H7 press fit + marine practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;groove_tool";
    tooling.tool_dia_mm       = boreFinishedDia;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.08;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(40.0, 20.0 + boreDepth * 1.2);
    tooling.extra = {
        { "marine_application", "rudder_gudgeon_pintle" },
        { "fit_class",          "H7" },
        { "removed_volume_mm3", volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rudder_gudgeon_pintle_bore: pintle={} bushing_od={} h7={} faces {}→{}",
                  in.pintle_dia_mm, in.bushing_od_mm, boreFinishedDia,
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

    // Geometric fallback: a single +Z bore with one slightly wider
    // concentric cylinder (the grease groove wall).
    int boreCyls   = 0;
    int grooveCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 4.0 && radius <= 25.0) ++boreCyls;
            else if (radius > 25.0 && radius <= 40.0) ++grooveCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1) {
        json recovered = { { "bushing_od_mm",   20.0 },
                           { "pintle_dia_mm",   12.0 } };
        json matched   = { { "source",       "geometric_bore_groove" },
                           { "bore_cyls",    boreCyls },
                           { "groove_cyls",  grooveCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::rudder_gudgeon_pintle_bore
