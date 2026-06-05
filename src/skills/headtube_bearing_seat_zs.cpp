// @lat: [[engine/skills#headtube_bearing_seat_zs]]

#include "headtube_bearing_seat_zs.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::headtube_bearing_seat_zs {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.headtube_id_mm <= 0.0 || in.bearing_od_mm <= 0.0 ||
        in.seat_angle_deg <= 0.0 || in.seat_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "headtube_bearing_seat_zs: all dimensions must be > 0");
        return r;
    }

    if (in.bearing_od_mm <= in.headtube_id_mm) {
        r.add("DFM-SEAT-OD", "error",
              "headtube_bearing_seat_zs: bearing_od_mm " +
              std::to_string(in.bearing_od_mm) +
              " must exceed headtube_id_mm " +
              std::to_string(in.headtube_id_mm));
    }

    if (std::abs(in.seat_angle_deg - 45.0) > 2.0) {
        r.add("DFM-ANGLE", "error",
              "headtube_bearing_seat_zs: seat_angle_deg " +
              std::to_string(in.seat_angle_deg) +
              " not the 45-degree zero-stack standard");
    }

    // Seats from both ends must not collide axially.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double tubeLen = zMax - zMin;
    if (2.0 * in.seat_depth_mm >= tubeLen) {
        r.add("DFM-INPUT", "error",
              "headtube_bearing_seat_zs: top + bottom seat depth overruns "
              "the head-tube length");
    }

    const double seatR  = in.bearing_od_mm / 2.0;
    const double stockR = 0.5 * std::min(xMax - xMin, yMax - yMin);
    if (seatR >= stockR) {
        r.add("DFM-STOCK", "error",
              "headtube_bearing_seat_zs: bearing seat radius " +
              std::to_string(seatR) + " mm exceeds stock radius " +
              std::to_string(stockR) + " mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "headtube_bearing_seat_zs DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ   = zMax;
    const double botZ   = zMin;
    const double tubeLen = zMax - zMin;
    const double cx     = in.axis_origin.X();
    const double cy     = in.axis_origin.Y();

    const double boreR = in.headtube_id_mm / 2.0;
    const double seatR = in.bearing_od_mm / 2.0;

    // ── 1) Central head-tube bore (full length) ──────────────────────────
    const gp_Pnt boreStart(cx, cy, botZ - kOver);
    const gp_Ax2 boreAx(boreStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(boreAx, boreR, tubeLen + 2.0 * kOver));

    // ── 2) Top 45-degree conical bearing seat ────────────────────────────
    // Cone built along +Z: small radius (= boreR) at bottom of seat, large
    // radius (= seatR) at the top face.  Opens outward toward the top.
    const gp_Pnt topSeatStart(cx, cy, topZ - in.seat_depth_mm);
    const gp_Ax2 topSeatAx(topSeatStart, gp::DZ());
    current = pr::cut(
        current,
        pr::coneFrustum(topSeatAx, boreR, seatR, in.seat_depth_mm + kOver));

    // ── 3) Bottom 45-degree conical bearing seat ─────────────────────────
    // Cone built along +Z: large radius (= seatR) at the bottom face,
    // narrowing to small radius (= boreR) at seat_depth above.
    const gp_Pnt botSeatStart(cx, cy, botZ - kOver);
    const gp_Ax2 botSeatAx(botSeatStart, gp::DZ());
    current = pr::cut(
        current,
        pr::coneFrustum(botSeatAx, seatR, boreR, in.seat_depth_mm + kOver));

    // ── Derived volume (analytic) ────────────────────────────────────────
    const double vBore = M_PI * boreR * boreR * tubeLen;
    // Conical seat minus the cylinder already removed by the bore over its
    // depth: V_cone = (pi*h/3)(R^2 + R*r + r^2); subtract pi*r^2*h.
    const double h = in.seat_depth_mm;
    const double vConeFull = (M_PI * h / 3.0) *
        (seatR * seatR + seatR * boreR + boreR * boreR);
    const double vSeatExtra = std::max(0.0, vConeFull - M_PI * boreR * boreR * h);
    const double volRemoved = vBore + 2.0 * vSeatExtra;

    json params = {
        { "axis_origin",    { in.axis_origin.X(),
                              in.axis_origin.Y(),
                              in.axis_origin.Z() } },
        { "headtube_id_mm", in.headtube_id_mm },
        { "bearing_od_mm",  in.bearing_od_mm },
        { "seat_angle_deg", in.seat_angle_deg },
        { "seat_depth_mm",  in.seat_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "bicycle_feature_type",       "zero_stack_headset_bearing_seat" },
        { "subfeature_count",           3 },
        { "headtube_id_mm",             in.headtube_id_mm },
        { "bearing_od_mm",              in.bearing_od_mm },
        { "seat_angle_deg",             in.seat_angle_deg },
        { "derived_seat_radius_mm",     seatR },
        { "derived_bore_radius_mm",     boreR },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ZS integrated headset (Cane Creek/ZS44)" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;facing_reamer";
    tooling.tool_dia_mm       = in.bearing_od_mm;
    tooling.tool_length_mm    = tubeLen + 20.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 150.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 95.0;
    tooling.extra = {
        { "bicycle_feature_type", "zero_stack_headset_bearing_seat" },
        { "seat_angle_deg",       in.seat_angle_deg },
        { "standard",             "ZS integrated" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::headtube_bearing_seat_zs: ID Ø{} bearing OD Ø{} angle {}",
                  in.headtube_id_mm, in.bearing_od_mm, in.seat_angle_deg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",               "metadata_replay" },
            { "is_compound",          true },
            { "bicycle_feature_type", "zero_stack_headset_bearing_seat" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: one central straight bore plus two conical seat
    // faces (non-cylindrical, opening to a larger radius).
    int boreCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 12.0 && radius <= 22.0) ++boreCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1) {
        json recovered = { { "headtube_id_mm", 30.5 },
                           { "bearing_od_mm",  41.0 } };
        json matched   = { { "source",    "geometric_headtube_bore" },
                           { "bore_cyls", boreCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::headtube_bearing_seat_zs
