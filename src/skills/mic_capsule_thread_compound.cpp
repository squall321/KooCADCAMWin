// @lat: [[engine/skills#mic_capsule_thread_compound]]

#include "mic_capsule_thread_compound.hpp"

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

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::mic_capsule_thread_compound {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

// ── Local non-overlap row for the audio-only 5/8"-27 UNS thread ─────────
// 5/8"-27 UNS is the universal microphone stand thread.  It is NOT in the
// standard UNC/UNF table (UNS = Unified National Special; non-standard pitch
// for a 5/8" OD).  We keep it as a local single-row constant.
namespace {
constexpr double k5_8_27_nominal_mm = 15.875;          // 5/8 inch
constexpr double k5_8_27_pitch_mm   = 25.4 / 27.0;     // 27 TPI ≈ 0.9407 mm
constexpr double k5_8_27_pilot_mm   = 14.4;            // ~60% engagement pilot
}  // namespace

double nominalDiaFor(const std::string& mount_style)
{
    if (mount_style == "5/8-27 UNS") return k5_8_27_nominal_mm;
    if (mount_style == "3/8-16 UNC") {
        const auto* s = tt::findUncUnf("3/8-16");
        return s ? s->nominal_dia_mm : 0.0;
    }
    if (mount_style == "1/4-20 UNC") {
        const auto* s = tt::findUncUnf("1/4-20");
        return s ? s->nominal_dia_mm : 0.0;
    }
    return 0.0;
}

double pitchFor(const std::string& mount_style)
{
    if (mount_style == "5/8-27 UNS") return k5_8_27_pitch_mm;
    if (mount_style == "3/8-16 UNC") return 25.4 / 16.0;   // 1.5875 mm
    if (mount_style == "1/4-20 UNC") return 25.4 / 20.0;   // 1.2700 mm
    return 0.0;
}

// Pilot (clearance) dia: 60% engagement.
static double pilotDiaFor(const std::string& mount_style)
{
    if (mount_style == "5/8-27 UNS") return k5_8_27_pilot_mm;
    if (mount_style == "3/8-16 UNC") {
        const auto* s = tt::findUncUnf("3/8-16");
        return s ? s->tap_pilot_dia_mm : 0.0;
    }
    if (mount_style == "1/4-20 UNC") {
        const auto* s = tt::findUncUnf("1/4-20");
        return s ? s->tap_pilot_dia_mm : 0.0;
    }
    return 0.0;
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mount_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "mic_capsule_thread_compound: mount_depth_mm must be > 0");
    }
    if (in.mount_style != "5/8-27 UNS"
        && in.mount_style != "3/8-16 UNC"
        && in.mount_style != "1/4-20 UNC") {
        r.add("DFM-MOUNT-STYLE", "error",
              "mic_capsule_thread_compound: mount_style '" + in.mount_style
              + "' must be '5/8-27 UNS' | '3/8-16 UNC' | '1/4-20 UNC'");
        (void)wp;
        return r;
    }
    const double pitch = pitchFor(in.mount_style);
    if (pitch > 0.0 && in.mount_depth_mm < std::max(4.0, 3.0 * pitch)) {
        r.add("DFM-DEPTH", "error",
              "mic_capsule_thread_compound: mount_depth "
              + std::to_string(in.mount_depth_mm)
              + " mm < 3 × pitch (" + std::to_string(3.0 * pitch)
              + " mm) — thread engagement insufficient");
    }
    if (!(std::abs(in.axis_dir.X()) < 1e-6 && std::abs(in.axis_dir.Y()) < 1e-6)) {
        r.add("DFM-AXIS", "error",
              "mic_capsule_thread_compound: only Z-aligned axis_dir is supported");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mic_capsule_thread_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double nominalDia = nominalDiaFor(in.mount_style);
    const double pitch      = pitchFor    (in.mount_style);
    const double pilotDia   = pilotDiaFor (in.mount_style);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kOver = 0.1;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;

    const bool fromTop = in.axis_dir.Z() < 0.0;
    const double entryZ = fromTop ? zMax : zMin;
    const gp_Dir axisDir = in.axis_dir;

    // ── 1) Cosmetic thread cylinder — major dia, thread_depth = mount_depth.
    const double threadR = nominalDia / 2.0;
    const gp_Pnt threadOrigin(cx, cy, entryZ + (fromTop ? kOver : -kOver));
    const gp_Ax2 threadAx(threadOrigin, axisDir);
    const TopoDS_Shape threadTool = pr::cylinder(threadAx, threadR,
        in.mount_depth_mm + 2.0 * kOver);
    TopoDS_Shape current = pr::cut(wp.shape(), threadTool);

    // ── 2) Clearance pilot — smaller dia, deeper by 2 * pitch (modeled the
    //       unthreaded extended clearance bore).
    const double pilotR = pilotDia / 2.0;
    const double pilotDepth = in.mount_depth_mm + 2.0 * pitch;
    const gp_Pnt pilotOrigin(cx, cy, entryZ + (fromTop ? kOver : -kOver));
    const gp_Ax2 pilotAx(pilotOrigin, axisDir);
    const TopoDS_Shape pilotTool = pr::cylinder(pilotAx, pilotR,
        pilotDepth + 2.0 * kOver);
    current = pr::cut(current, pilotTool);

    const double vThread = M_PI * (threadR * threadR - pilotR * pilotR)
                         * in.mount_depth_mm;
    const double vPilot  = M_PI * pilotR * pilotR * pilotDepth;
    const double volRemoved = std::max(0.0, vThread) + vPilot;

    json params = {
        { "center_x_mm",     cx },
        { "center_y_mm",     cy },
        { "axis_dir",        { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "mount_style",     in.mount_style },
        { "mount_depth_mm",  in.mount_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "audio_feature_type",  "mic_capsule_thread" },
        { "subfeature_count",    2 },
        { "mount_style",         in.mount_style },
        { "thread_nominal_dia_mm", nominalDia },
        { "thread_pitch_mm",     pitch },
        { "pilot_dia_mm",        pilotDia },
        { "mount_depth_mm",      in.mount_depth_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;tap";
    tooling.tool_dia_mm       = nominalDia;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(8.0, in.mount_depth_mm * 0.5 + 5.0);
    tooling.extra = {
        { "standard",       "5/8-27 UNS universal mic stand thread / UNC adapter" },
        { "thread_pitch_mm", pitch },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::mic_capsule_thread_compound: mount={} depth={}",
                  in.mount_style, in.mount_depth_mm);

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

    // Geometric: two coaxial cylinders along Z.
    int axialCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9) ++axialCyls;
        } catch (...) {}
    }
    if (axialCyls >= 2) {
        json recovered = { { "candidate", "mic_capsule_thread_pattern" } };
        json matched   = { { "source", "geometric_coaxial_cyl_pair" },
                           { "axial_cyl_count", axialCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.5, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mic_capsule_thread_compound
