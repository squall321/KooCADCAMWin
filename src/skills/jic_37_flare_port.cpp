// @lat: [[engine/skills#jic_37_flare_port]]

#include "jic_37_flare_port.hpp"

#include "Workpiece.hpp"
#include "_hydraulic_ports.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::jic_37_flare_port {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hyd_ports;
using nlohmann::json;

namespace {
constexpr double kOverhang = 0.05;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tube_size.empty()) {
        r.add("DFM-JIC-CODE", "error",
              "jic_37_flare_port: tube_size is empty");
        return r;
    }
    const ht::JicFlareSpec* s = ht::findJicFlare(in.tube_size);
    if (!s) {
        r.add("DFM-JIC-CODE", "error",
              "jic_37_flare_port: unknown tube_size '" + in.tube_size +
              "' (supported: -4, -6, -8, -12, -16)");
        return r;
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-JIC-FACE", "error",
              "jic_37_flare_port: face_id datum unresolved");
        return r;
    }
    if (!wp.isFacePlanar(*faceId)) {
        r.add("DFM-JIC-FACE", "error",
              "jic_37_flare_port: target face is not planar");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double minSpan = 1.5 * s->flare_seat_od_mm;
    if (xMax - xMin < minSpan || yMax - yMin < minSpan) {
        r.add("DFM-JIC-MAT", "error",
              "jic_37_flare_port: face span < 1.5 × flare_seat_od (need " +
              std::to_string(minSpan) + " mm)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "jic_37_flare_port DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::JicFlareSpec* s = ht::findJicFlare(in.tube_size);
    if (!s) throw SkillError("jic_37_flare_port: tube_size lookup failed");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("jic_37_flare_port: face_id unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir = in.axis_dir;
    gp_Pnt entryStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        entryStart = gp_Pnt(in.center_x_mm, in.center_y_mm,
                            (adir.Z() < 0) ? zMax + kOverhang : zMin - kOverhang);
    } else {
        entryStart = gp_Pnt(in.center_x_mm, in.center_y_mm,
                            (zMin + zMax) / 2.0 - adir.Z() * kOverhang);
    }

    const double boreR     = s->bore_dia_mm     / 2.0;
    const double threadR   = s->thread_major_mm / 2.0;
    const double flareTopR = s->flare_seat_od_mm / 2.0;
    const double seatD     = s->seat_depth_mm;
    const double boreDepth = (in.bore_depth_mm > 0.0) ? in.bore_depth_mm
                                                      : s->bore_dia_mm * 2.0 + seatD;
    const double threadDepth = seatD + 1.5;  // straight UN/UNF above seat

    // ── Sub-cut #1: straight central flow bore ───────────────────────────
    const gp_Ax2 boreAx(entryStart, adir);
    const TopoDS_Shape mainBore =
        pr::cylinder(boreAx, boreR, boreDepth + kOverhang);

    // ── Sub-cut #2: 37° conical seat ─────────────────────────────────────
    // Cone is wide at the face (flareTopR) and narrows to boreR over seatD.
    const gp_Ax2 coneAx(entryStart, adir);
    const TopoDS_Shape flareCone =
        pr::coneFrustum(coneAx, flareTopR, boreR, seatD + kOverhang);

    // ── Sub-cut #3: UN/UNF clearance cylinder above the seat ─────────────
    // The threaded portion is just above (toward face) the conical seat —
    // since the cone already occupies that volume up to seatD, the thread
    // clearance is the cylinder from entry to seatD with radius = threadR.
    const gp_Ax2 thrdAx(entryStart, adir);
    const TopoDS_Shape threadBore =
        pr::cylinder(thrdAx, threadR, threadDepth + kOverhang);

    TopoDS_Shape unified = pr::fuse(mainBore, flareCone);
    unified = pr::fuse(unified, threadBore);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    const double volBore = M_PI * boreR * boreR * boreDepth;
    const double volCone = M_PI / 3.0 * seatD
                           * (flareTopR * flareTopR
                              + flareTopR * boreR + boreR * boreR);
    const double volThread = M_PI * threadR * threadR * threadDepth
                             - M_PI * boreR * boreR * threadDepth;
    const double volRemoved = volBore + std::max(0.0, volCone)
                                      + std::max(0.0, volThread);

    json params = {
        { "face_id_resolved", *faceId },
        { "center_x_mm",      in.center_x_mm },
        { "center_y_mm",      in.center_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "tube_size",        in.tube_size },
        { "bore_depth_mm",    in.bore_depth_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "port_standard",           "JIC_37_SAE_J514" },
        { "tube_size",               in.tube_size },
        { "subfeature_count",        3 },
        { "thread_major_mm",         s->thread_major_mm },
        { "thread_pitch_mm",         s->thread_pitch_mm },
        { "bore_dia_mm",             s->bore_dia_mm },
        { "port_dia_mm",             s->bore_dia_mm },
        { "flare_seat_od_mm",        s->flare_seat_od_mm },
        { "flare_half_angle_deg",    ht::kJicFlareHalfAngleDeg },
        { "seat_depth_mm",           s->seat_depth_mm },
        { "derived_volume_removed",  volRemoved },
        { "axis_dir",                { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;cone_tool;tap";
    tooling.tool_dia_mm       = s->bore_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.est_cycle_time_s  = std::max(8.0, boreDepth * 0.5 + 5.0);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->bore_dia_mm },
              { "depth_mm", boreDepth },
              { "role", "central_flow_bore" } },
            { { "tool_type", "cone_tool" },
              { "half_angle_deg", ht::kJicFlareHalfAngleDeg },
              { "od_mm", s->flare_seat_od_mm },
              { "depth_mm", s->seat_depth_mm },
              { "role", "37deg_flare_seat" } },
            { { "tool_type", "tap" },
              { "thread_major_mm", s->thread_major_mm },
              { "pitch_mm", s->thread_pitch_mm },
              { "role", "UN_UNF_straight_thread" } },
        } },
        { "standard", "SAE J514 JIC 37° flare-seat port" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::jic_37_flare_port applied: {} bore={} flareOD={}",
                  in.tube_size, s->bore_dia_mm, s->flare_seat_od_mm);

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
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::jic_37_flare_port
