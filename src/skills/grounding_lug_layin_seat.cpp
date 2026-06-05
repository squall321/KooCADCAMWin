// @lat: [[engine/skills#grounding_lug_layin_seat]]

#include "grounding_lug_layin_seat.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::grounding_lug_layin_seat {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.seat_len_mm <= 0.0 || in.seat_wid_mm <= 0.0 ||
        in.seat_depth_mm <= 0.0 || in.conductor_groove_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "grounding_lug_layin_seat: all dimensions must be > 0");
    }
    if (in.set_screw_thread_key.empty()) {
        r.add("DFM-THREAD", "error",
              "grounding_lug_layin_seat: set_screw_thread_key is empty");
    } else if (!tt::findMetric(in.set_screw_thread_key)) {
        r.add("DFM-THREAD", "error",
              "grounding_lug_layin_seat: set_screw_thread_key '" +
              in.set_screw_thread_key +
              "' not in central metric thread table");
    }
    if (in.conductor_groove_dia_mm > 0.0 && in.seat_wid_mm > 0.0 &&
        in.conductor_groove_dia_mm >= in.seat_wid_mm) {
        r.add("DFM-GROOVE", "error",
              "grounding_lug_layin_seat: conductor_groove_dia_mm " +
              std::to_string(in.conductor_groove_dia_mm) +
              " must be < seat_wid_mm " + std::to_string(in.seat_wid_mm));
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "grounding_lug_layin_seat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* thr = tt::findMetric(in.set_screw_thread_key);
    if (!thr) throw SkillError("grounding_lug_layin_seat: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Lug seat pocket: box cut DOWN from top face ──────────────────
    const double sx = in.seat_len_mm;   // length along X
    const double sy = in.seat_wid_mm;    // width along Y
    const double sd = in.seat_depth_mm;
    const gp_Pnt seatOrigin(cx - sx / 2.0, cy - sy / 2.0, topZ - sd);
    const gp_Ax2 seatAx(seatOrigin, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::box(seatAx, sx, sy, sd + kOver));

    // ── 2) Lay-in conductor groove: horizontal cylinder along +X ────────
    // Axis runs along +X at the seat floor centreline; the cylinder removes
    // a round channel for the bare conductor laid into the seat.
    const double grR = in.conductor_groove_dia_mm / 2.0;
    const double grLen = sx + 2.0 * kOver;
    const gp_Pnt grStart(cx - sx / 2.0 - kOver, cy, topZ - sd);
    const gp_Ax2 grAx(grStart, gp::DX());
    current = pr::cut(current, pr::cylinder(grAx, grR, grLen));

    // ── 3) Set-screw clearance hole: vertical M-thread clearance cyl ────
    const double clrR = thr->clearance_medium_mm / 2.0;
    const double clrDepth = sd + std::min(6.0, (zMax - zMin) * 0.4);
    const gp_Pnt clrStart(cx, cy, topZ - clrDepth);
    const gp_Ax2 clrAx(clrStart, gp::DZ());
    current = pr::cut(current, pr::cylinder(clrAx, clrR, clrDepth + kOver));

    const double vSeat = sx * sy * sd;
    const double vGroove = M_PI * grR * grR * grLen * 0.5;  // ~half embedded
    const double belowFloor = std::max(0.0, clrDepth - sd);
    const double vScrew = M_PI * clrR * clrR * belowFloor;
    const double volRemoved = vSeat + vGroove + vScrew;

    json params = {
        { "center_xy",               { in.center_xy.X(), in.center_xy.Y(), in.center_xy.Z() } },
        { "seat_len_mm",             in.seat_len_mm },
        { "seat_wid_mm",             in.seat_wid_mm },
        { "seat_depth_mm",           in.seat_depth_mm },
        { "conductor_groove_dia_mm", in.conductor_groove_dia_mm },
        { "set_screw_thread_key",    in.set_screw_thread_key },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "solar_feature_type",         "grounding_lug_layin_seat" },
        { "subfeature_count",           3 },
        { "set_screw_thread_key",       in.set_screw_thread_key },
        { "derived_set_screw_clr_mm",   thr->clearance_medium_mm },
        { "derived_groove_dia_mm",      in.conductor_groove_dia_mm },
        { "derived_seat_len_mm",        in.seat_len_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "UL 467 / NEC 690.43" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;ball_mill;drill";
    tooling.tool_dia_mm       = in.conductor_groove_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(20.0, in.seat_len_mm / 2.0 + 8.0);
    tooling.extra = {
        { "solar_application", "lay_in_grounding_lug" },
        { "standard",          "UL 467" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::grounding_lug_layin_seat: seat {}x{} screw={}",
                  in.seat_len_mm, in.seat_wid_mm, in.set_screw_thread_key);

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

    // Geometric: a vertical set-screw cylinder plus a horizontal groove
    // cylinder (axis along X) near the same XY.
    int vertCyls = 0;
    int horizCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9) ++vertCyls;
            else if (std::abs(d.X()) > 0.9) ++horizCyls;
        } catch (...) {}
    }
    if (vertCyls >= 1 && horizCyls >= 1) {
        json recovered = { { "seat_len_mm",             24.0 },
                           { "seat_wid_mm",             12.0 },
                           { "seat_depth_mm",           2.0 },
                           { "conductor_groove_dia_mm", 6.0 },
                           { "set_screw_thread_key",    "M6" } };
        json matched = { { "source", "geometric_lug_groove_pattern" },
                         { "vert_cyls", vertCyls },
                         { "horiz_cyls", horizCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::grounding_lug_layin_seat
