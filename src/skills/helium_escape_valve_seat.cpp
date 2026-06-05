// @lat: [[engine/skills#helium_escape_valve_seat]]

#include "helium_escape_valve_seat.hpp"

#include "Workpiece.hpp"
#include "_as568_table.hpp"
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

namespace koocadcam::skill::helium_escape_valve_seat {

namespace pr = koocadcam::engine::prim;
namespace as = koocadcam::skill::as568;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.valve_bore_dia_mm <= 0.0 || in.spring_seat_dia_mm <= 0.0 ||
        in.seat_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "helium_escape_valve_seat: all dimensions must be > 0");
        return r;
    }

    const as::DashSpec* ring = as::findDash(in.o_ring_size_key);
    if (!ring) {
        r.add("DFM-AS568", "error",
              "helium_escape_valve_seat: o_ring_size_key '" +
              in.o_ring_size_key + "' not in central AS568 table");
    }

    if (in.spring_seat_dia_mm <= in.valve_bore_dia_mm) {
        r.add("DFM-SEAT", "error",
              "helium_escape_valve_seat: spring_seat_dia " +
              std::to_string(in.spring_seat_dia_mm) +
              " mm must exceed valve_bore_dia " +
              std::to_string(in.valve_bore_dia_mm) + " mm");
    }

    if (in.seat_depth_mm > 8.0) {
        r.add("DFM-DEPTH", "error",
              "helium_escape_valve_seat: seat_depth " +
              std::to_string(in.seat_depth_mm) +
              " mm exceeds 8.0 mm practical limit");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "helium_escape_valve_seat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const as::DashSpec* ring = as::findDash(in.o_ring_size_key);
    if (!ring) throw SkillError("helium_escape_valve_seat: AS568 lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) HEV bore (narrow through-ish cylinder cut DOWN from top) ──────
    const double boreR = in.valve_bore_dia_mm / 2.0;
    const double boreDepth = std::max(in.seat_depth_mm + 4.0,
                                      (zMax - zMin) * 0.7);
    const gp_Pnt boreStart(cx, cy, topZ - boreDepth);
    const gp_Ax2 boreAx(boreStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(), pr::cylinder(boreAx, boreR, boreDepth + kOver));

    // ── 2) Spring-seat counterbore (wider cylinder, seat_depth deep) ─────
    const double seatR = in.spring_seat_dia_mm / 2.0;
    const gp_Pnt seatStart(cx, cy, topZ - in.seat_depth_mm);
    const gp_Ax2 seatAx(seatStart, gp::DZ());
    const TopoDS_Shape seatTool = pr::cylinder(
        seatAx, seatR, in.seat_depth_mm + kOver);
    current = pr::cut(current, seatTool);

    // ── 3) AS568 O-ring groove (annular ring around seat wall) ───────────
    const double grooveOd = in.spring_seat_dia_mm + 2.0 * ring->radial_groove_inset_mm
                            + ring->cross_section_mm;
    const double grooveId = in.spring_seat_dia_mm - 0.04;   // bias from seat wall
    const double grooveDepth = ring->groove_depth_mm;
    const gp_Pnt grooveStart(cx, cy, topZ - grooveDepth);
    const gp_Ax2 grooveAx(grooveStart, gp::DZ());
    const TopoDS_Shape grooveTool = pr::annularRing(
        grooveAx, grooveOd / 2.0, grooveId / 2.0, grooveDepth + kOver);
    current = pr::cut(current, grooveTool);

    const double vBore = M_PI * boreR * boreR * boreDepth;
    const double vSeat = M_PI * (seatR * seatR - boreR * boreR) * in.seat_depth_mm;
    const double vGroove = M_PI *
        ((grooveOd * grooveOd - grooveId * grooveId) / 4.0) * grooveDepth;
    const double volRemoved = vBore + std::max(0.0, vSeat) + vGroove;

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "valve_bore_dia_mm",  in.valve_bore_dia_mm },
        { "spring_seat_dia_mm", in.spring_seat_dia_mm },
        { "seat_depth_mm",      in.seat_depth_mm },
        { "o_ring_size_key",    in.o_ring_size_key },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "watch_feature_type",         "helium_escape_valve_seat" },
        { "subfeature_count",           3 },
        { "o_ring_size_key",            in.o_ring_size_key },
        { "derived_bore_dia_mm",        in.valve_bore_dia_mm },
        { "derived_seat_dia_mm",        in.spring_seat_dia_mm },
        { "derived_groove_od_mm",       grooveOd },
        { "derived_groove_id_mm",       grooveId },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;counterbore;groove_tool";
    tooling.tool_dia_mm       = in.valve_bore_dia_mm;
    tooling.tool_length_mm    = boreDepth + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(20.0, volRemoved / 50.0);
    tooling.extra = {
        { "watch_application", "helium_escape_valve" },
        { "o_ring_standard",   "AS568" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::helium_escape_valve_seat: bore={} seat={} oring={}",
                  in.valve_bore_dia_mm, in.spring_seat_dia_mm, in.o_ring_size_key);

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

    // Geometric fallback: a narrow bore cyl + a wider seat cyl concentric on
    // a vertical axis indicates a valve seat family.
    int boreCyls = 0;
    int seatCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 0.5 && radius <= 2.0) ++boreCyls;
            else if (radius > 2.0 && radius <= 6.0) ++seatCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1 && seatCyls >= 1) {
        json recovered = { { "valve_bore_dia_mm",  2.0 },
                           { "spring_seat_dia_mm", 5.0 },
                           { "seat_depth_mm",      3.0 },
                           { "o_ring_size_key",    "-011" } };
        json matched   = { { "source",    "geometric_valve_seat_pattern" },
                           { "bore_cyls", boreCyls },
                           { "seat_cyls", seatCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::helium_escape_valve_seat
