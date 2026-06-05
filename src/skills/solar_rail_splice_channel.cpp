// @lat: [[engine/skills#solar_rail_splice_channel]]

#include "solar_rail_splice_channel.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::solar_rail_splice_channel {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.channel_width_mm <= 0.0 || in.channel_depth_mm <= 0.0 ||
        in.channel_length_mm <= 0.0 || in.screw_pilot_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "solar_rail_splice_channel: all dimensions must be > 0");
    }
    if (in.screw_count < 1 || in.screw_count > 12) {
        r.add("DFM-SCREWS", "error",
              "solar_rail_splice_channel: screw_count " +
              std::to_string(in.screw_count) + " out of range [1, 12]");
    }
    if (in.screw_pilot_dia_mm > 0.0 && in.channel_width_mm > 0.0 &&
        in.screw_pilot_dia_mm >= in.channel_width_mm) {
        r.add("DFM-PILOT", "error",
              "solar_rail_splice_channel: screw_pilot_dia_mm " +
              std::to_string(in.screw_pilot_dia_mm) +
              " must be < channel_width_mm " +
              std::to_string(in.channel_width_mm));
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "solar_rail_splice_channel DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Splice channel: box cut DOWN from the top face ───────────────
    const double sx = in.channel_length_mm;   // length along X
    const double sy = in.channel_width_mm;      // width along Y
    const double sd = in.channel_depth_mm;
    const gp_Pnt chOrigin(cx - sx / 2.0, cy - sy / 2.0, topZ - sd);
    const gp_Ax2 chAx(chOrigin, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::box(chAx, sx, sy, sd + kOver));

    // ── N) screw pilot holes: cylinders spaced along channel length (X) ─
    const double pilotR = in.screw_pilot_dia_mm / 2.0;
    const double pilotDepth = sd + std::min(6.0, (zMax - zMin) * 0.5);
    const double pitch = sx / static_cast<double>(in.screw_count);
    for (int i = 0; i < in.screw_count; ++i) {
        const double pcx = (cx - sx / 2.0) + pitch * (i + 0.5);
        const gp_Pnt pStart(pcx, cy, topZ - pilotDepth);
        const gp_Ax2 pAx(pStart, gp::DZ());
        current = pr::cut(current,
                          pr::cylinder(pAx, pilotR, pilotDepth + kOver));
    }

    const double vChannel = sx * sy * sd;
    // Pilots cut additional material below the channel floor.
    const double belowFloor = std::max(0.0, pilotDepth - sd);
    const double vPilots =
        static_cast<double>(in.screw_count) * M_PI * pilotR * pilotR * belowFloor;
    const double volRemoved = vChannel + vPilots;

    json params = {
        { "center_xy",          { in.center_xy.X(), in.center_xy.Y(), in.center_xy.Z() } },
        { "channel_width_mm",   in.channel_width_mm },
        { "channel_depth_mm",   in.channel_depth_mm },
        { "channel_length_mm",  in.channel_length_mm },
        { "screw_count",        in.screw_count },
        { "screw_pilot_dia_mm", in.screw_pilot_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "solar_feature_type",         "rail_splice_channel" },
        { "subfeature_count",           1 + in.screw_count },
        { "screw_count",                in.screw_count },
        { "derived_channel_width_mm",   in.channel_width_mm },
        { "derived_channel_depth_mm",   in.channel_depth_mm },
        { "derived_pilot_dia_mm",       in.screw_pilot_dia_mm },
        { "derived_pilot_pitch_mm",     pitch },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "UL 2703" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "slot_mill;drill";
    tooling.tool_dia_mm       = in.channel_width_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  =
        std::max(25.0, in.channel_length_mm / 2.0 + in.screw_count * 3.0);
    tooling.extra = {
        { "solar_application", "rail_splice" },
        { "standard",          "UL 2703" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::solar_rail_splice_channel: ch {}x{} screws={}",
                  in.channel_length_mm, in.channel_width_mm, in.screw_count);

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

    // Geometric: a row of small coaxial-Z pilot cylinders below a channel.
    int pilotCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 1.0 && radius <= 4.0) ++pilotCyls;
        } catch (...) {}
    }
    if (pilotCyls >= 2) {
        json recovered = { { "channel_width_mm",   12.0 },
                           { "channel_depth_mm",   5.0 },
                           { "channel_length_mm",  60.0 },
                           { "screw_count",        pilotCyls },
                           { "screw_pilot_dia_mm", 3.5 } };
        json matched = { { "source", "geometric_pilot_row" },
                         { "pilot_cyls", pilotCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::solar_rail_splice_channel
