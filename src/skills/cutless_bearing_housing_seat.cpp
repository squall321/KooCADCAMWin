// @lat: [[engine/skills#cutless_bearing_housing_seat]]

#include "cutless_bearing_housing_seat.hpp"

#include "Workpiece.hpp"
#include "_iso286_fits.hpp"
#include "_iso_thread_table.hpp"
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

namespace koocadcam::skill::cutless_bearing_housing_seat {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
constexpr int    kSetScrewCount = 2;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bearing_od_mm <= 0.0 || in.housing_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cutless_bearing_housing_seat: bearing_od_mm and "
              "housing_length_mm must be > 0");
    }

    if (in.set_screw_thread_key.empty()) {
        r.add("DFM-M-THREAD", "error",
              "cutless_bearing_housing_seat: set_screw_thread_key is empty");
    } else if (!tt::findMetric(in.set_screw_thread_key)) {
        r.add("DFM-M-THREAD", "error",
              "cutless_bearing_housing_seat: set_screw_thread_key '" +
              in.set_screw_thread_key +
              "' not present in central _iso_thread_table.hpp");
    }

    if (in.groove_count < 1 || in.groove_count > 6) {
        r.add("DFM-MARINE-GROOVE", "error",
              "cutless_bearing_housing_seat: groove_count (" +
              std::to_string(in.groove_count) +
              ") must be in [1, 6]");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cutless_bearing_housing_seat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* mSpec = tt::findMetric(in.set_screw_thread_key);
    if (!mSpec) throw SkillError("cutless_bearing_housing_seat: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.axis_origin.X();
    const double cy = in.axis_origin.Y();

    // ── 1) Bearing housing bore — P7 hole sized to the bearing OD ──────
    // Press fit: hole held at the P7 limit (negative deviation) so the
    // bearing shell is captive.
    const double boreFinishedDia = iso286::p7_max_mm(in.bearing_od_mm);
    const double boreR = boreFinishedDia / 2.0;
    const double boreLen = std::min(in.housing_length_mm, (zMax - zMin));
    const gp_Ax2 boreAx(gp_Pnt(cx, cy, topZ - boreLen), gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(),
                                   pr::cylinder(boreAx, boreR, boreLen + kOver));

    // ── 2 & 3) Two radial set-screw clearance holes (drilled in -X) ────
    const double setClr = mSpec->clearance_medium_mm;
    const double setR   = setClr / 2.0;
    const double radialDepth = (xMax - cx) + kOver;        // from +X face to axis
    // Two set screws spaced along the bore axis.
    const double z1 = topZ - boreLen * 0.25;
    const double z2 = topZ - boreLen * 0.75;
    for (int i = 0; i < kSetScrewCount; ++i) {
        const double sz = (i == 0 ? z1 : z2);
        // Drill axis points -X (into the part) starting just outside +X face.
        const gp_Ax2 setAx(gp_Pnt(xMax + kOver, cy, sz), gp_Dir(-1.0, 0.0, 0.0));
        current = pr::cut(current, pr::cylinder(setAx, setR, radialDepth + kOver));
    }

    // ── 4..) Internal water-relief grooves (annular rings in the bore) ──
    const double grooveW  = 2.5;
    const double grooveOD = boreFinishedDia + 2.0;        // 1 mm radial relief
    const double grooveID = boreFinishedDia;              // starts at bore wall
    for (int i = 0; i < in.groove_count; ++i) {
        const double frac = static_cast<double>(i + 1) /
                            static_cast<double>(in.groove_count + 1);
        const double gz = topZ - boreLen * frac - grooveW / 2.0;
        const gp_Ax2 grAx(gp_Pnt(cx, cy, gz), gp::DZ());
        current = pr::cut(current,
                          pr::annularRing(grAx, grooveOD / 2.0,
                                          grooveID / 2.0, grooveW));
    }

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double vBore   = M_PI * boreR * boreR * boreLen;
    const double vSets   = static_cast<double>(kSetScrewCount) *
                           M_PI * setR * setR * radialDepth;
    const double vGroove = static_cast<double>(in.groove_count) * M_PI *
        ((grooveOD * 0.5) * (grooveOD * 0.5) -
         (grooveID * 0.5) * (grooveID * 0.5)) * grooveW;
    const double volRemoved = vBore + vSets + vGroove;

    const int subfeatureCount = 1 + kSetScrewCount + in.groove_count;

    json params = {
        { "axis_origin",          { in.axis_origin.X(),
                                    in.axis_origin.Y(),
                                    in.axis_origin.Z() } },
        { "bearing_od_mm",        in.bearing_od_mm },
        { "housing_length_mm",    in.housing_length_mm },
        { "set_screw_thread_key", in.set_screw_thread_key },
        { "groove_count",         in.groove_count },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "marine_feature_type",        "cutless_bearing_housing_seat" },
        { "subfeature_count",           subfeatureCount },
        { "bearing_od_mm",              in.bearing_od_mm },
        { "housing_length_mm",          in.housing_length_mm },
        { "set_screw_thread",           in.set_screw_thread_key },
        { "set_screw_count",            kSetScrewCount },
        { "groove_count",               in.groove_count },
        { "derived_bore_p7_max_mm",     boreFinishedDia },
        { "derived_set_screw_hole_dia_mm", setClr },
        { "derived_groove_od_mm",       grooveOD },
        { "derived_groove_width_mm",    grooveW },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 286 P7 press fit + marine practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;drill;groove_tool";
    tooling.tool_dia_mm       = boreFinishedDia;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.08;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 50.0 + 8.0 * static_cast<double>(in.groove_count);
    tooling.extra = {
        { "marine_application", "cutless_bearing_housing" },
        { "fit_class",          "P7" },
        { "removed_volume_mm3", volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cutless_bearing_housing_seat: od={} p7={} grooves={} faces {}→{}",
                  in.bearing_od_mm, boreFinishedDia, in.groove_count,
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

    // Geometric fallback: a large +Z housing bore plus small radial (X-axis)
    // set-screw cylinders.
    int boreCyls = 0;
    int setCyls  = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            const double radius = c.Radius();
            if (std::abs(c.Axis().Direction().Z()) > 0.9 && radius >= 8.0) ++boreCyls;
            else if (std::abs(c.Axis().Direction().X()) > 0.9 && radius < 8.0) ++setCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1) {
        json recovered = { { "bearing_od_mm", 30.0 },
                           { "groove_count",  2 } };
        json matched   = { { "source",     "geometric_housing_seat" },
                           { "bore_cyls",  boreCyls },
                           { "set_cyls",   setCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::cutless_bearing_housing_seat
