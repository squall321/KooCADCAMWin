// @lat: [[engine/skills#tracker_slew_pivot_bore]]

#include "tracker_slew_pivot_bore.hpp"

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

namespace koocadcam::skill::tracker_slew_pivot_bore {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pivot_bore_dia_mm <= 0.0 || in.bushing_od_mm <= 0.0 ||
        in.bore_depth_mm <= 0.0 || in.stop_pin_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tracker_slew_pivot_bore: all dimensions must be > 0");
    }
    if (in.grease_thread_key.empty()) {
        r.add("DFM-THREAD", "error",
              "tracker_slew_pivot_bore: grease_thread_key is empty");
    } else if (!tt::findMetric(in.grease_thread_key)) {
        r.add("DFM-THREAD", "error",
              "tracker_slew_pivot_bore: grease_thread_key '" +
              in.grease_thread_key +
              "' not in central metric thread table");
    }
    if (in.bushing_od_mm > 0.0 && in.pivot_bore_dia_mm > 0.0 &&
        in.bushing_od_mm <= in.pivot_bore_dia_mm) {
        r.add("DFM-SEAT", "error",
              "tracker_slew_pivot_bore: bushing_od_mm " +
              std::to_string(in.bushing_od_mm) +
              " must be > pivot_bore_dia_mm " +
              std::to_string(in.pivot_bore_dia_mm));
    }
    if (in.bushing_od_mm > 0.0 && !iso286::findBand(in.bushing_od_mm)) {
        r.add("DFM-H7RANGE", "error",
              "tracker_slew_pivot_bore: bushing_od_mm " +
              std::to_string(in.bushing_od_mm) +
              " outside ISO 286-1 band coverage (<= 100 mm)");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tracker_slew_pivot_bore DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* thr = tt::findMetric(in.grease_thread_key);
    if (!thr) throw SkillError("tracker_slew_pivot_bore: thread lookup failed");

    // H7 max diameter for the bushing seat (hole-basis: bushing OD nominal).
    const double seatH7max = iso286::h7_max_mm(in.bushing_od_mm);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.axis_origin.X();
    const double cy = in.axis_origin.Y();

    // ── 1) Pivot bore + H7 bushing seat ─────────────────────────────────
    // The pivot bore goes full depth; the bushing seat is a wider counterbore
    // (H7-max diameter) at the entry, depth = ~40 % of bore depth.
    const double boreR = in.pivot_bore_dia_mm / 2.0;
    const double boreDepth = in.bore_depth_mm;
    const gp_Pnt boreStart(cx, cy, topZ - boreDepth);
    const gp_Ax2 boreAx(boreStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(boreAx, boreR, boreDepth + kOver));

    const double seatR = seatH7max / 2.0;
    const double seatDepth = std::max(2.0, boreDepth * 0.4);
    const gp_Pnt seatStart(cx, cy, topZ - seatDepth);
    const gp_Ax2 seatAx(seatStart, gp::DZ());
    current = pr::cut(current, pr::cylinder(seatAx, seatR, seatDepth + kOver));

    // ── 2) Grease-fitting tapped hole (M-thread pilot, vertical from top) ─
    const double greaseR = thr->tap_pilot_dia_mm / 2.0;
    const double greaseDepth = std::min(boreDepth * 0.6, (zMax - zMin) * 0.5);
    const double gx = cx + seatR + greaseR + 2.0;
    const gp_Pnt greaseStart(gx, cy, topZ - greaseDepth);
    const gp_Ax2 greaseAx(greaseStart, gp::DZ());
    current = pr::cut(current, pr::cylinder(greaseAx, greaseR, greaseDepth + kOver));

    // ── 3) Stop-pin hole (plain cylinder, vertical) ─────────────────────
    const double pinR = in.stop_pin_dia_mm / 2.0;
    const double pinDepth = std::min(boreDepth, (zMax - zMin) * 0.6);
    const double px = cx - seatR - pinR - 2.0;
    const gp_Pnt pinStart(px, cy, topZ - pinDepth);
    const gp_Ax2 pinAx(pinStart, gp::DZ());
    current = pr::cut(current, pr::cylinder(pinAx, pinR, pinDepth + kOver));

    const double vBore = M_PI * boreR * boreR * boreDepth;
    const double vSeat =
        M_PI * (seatR * seatR - boreR * boreR) * seatDepth;
    const double vGrease = M_PI * greaseR * greaseR * greaseDepth;
    const double vPin = M_PI * pinR * pinR * pinDepth;
    const double volRemoved = vBore + std::max(0.0, vSeat) + vGrease + vPin;

    json params = {
        { "axis_origin",       { in.axis_origin.X(), in.axis_origin.Y(), in.axis_origin.Z() } },
        { "pivot_bore_dia_mm", in.pivot_bore_dia_mm },
        { "bushing_od_mm",     in.bushing_od_mm },
        { "bore_depth_mm",     in.bore_depth_mm },
        { "grease_thread_key", in.grease_thread_key },
        { "stop_pin_dia_mm",   in.stop_pin_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "solar_feature_type",         "tracker_slew_pivot_bore" },
        { "subfeature_count",           3 },
        { "grease_thread_key",          in.grease_thread_key },
        { "derived_seat_h7_max_mm",     seatH7max },
        { "derived_pivot_bore_dia_mm",  in.pivot_bore_dia_mm },
        { "derived_grease_pilot_mm",    thr->tap_pilot_dia_mm },
        { "derived_stop_pin_dia_mm",    in.stop_pin_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 286-1 (H7)" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;reamer;drill;tap";
    tooling.tool_dia_mm       = in.pivot_bore_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 120.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(40.0, in.bore_depth_mm * 1.5 + 20.0);
    tooling.extra = {
        { "solar_application", "tracker_pivot_bearing" },
        { "standard",          "ISO 286-1" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::tracker_slew_pivot_bore: bore {} seat H7max {:.3f} grease {}",
                  in.pivot_bore_dia_mm, seatH7max, in.grease_thread_key);

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

    // Geometric: a large pivot bore (concentric seat cyls) plus two smaller
    // vertical holes (grease + stop pin).
    int bigCyls = 0;
    int smallCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 10.0) ++bigCyls;
            else if (radius >= 1.5 && radius < 10.0) ++smallCyls;
        } catch (...) {}
    }
    if (bigCyls >= 1 && smallCyls >= 1) {
        json recovered = { { "pivot_bore_dia_mm", 24.0 },
                           { "bushing_od_mm",     30.0 },
                           { "bore_depth_mm",     18.0 },
                           { "grease_thread_key", "M8" },
                           { "stop_pin_dia_mm",   8.0 } };
        json matched = { { "source", "geometric_pivot_bore_pattern" },
                         { "big_cyls", bigCyls },
                         { "small_cyls", smallCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::tracker_slew_pivot_bore
