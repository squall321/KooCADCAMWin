// @lat: [[engine/skills#bilge_drain_threaded_compound]]

#include "bilge_drain_threaded_compound.hpp"

#include "Workpiece.hpp"
#include "_hydraulic_ports.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::bilge_drain_threaded_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.spotface_dia_mm <= 0.0 || in.spotface_depth_mm <= 0.0 ||
        in.drain_dia_mm <= 0.0 || in.plate_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bilge_drain_threaded_compound: dimensions must be > 0");
    }

    const auto* gSpec = hyd_ports::findBspG(in.thread_size_key);
    if (!gSpec) {
        r.add("DFM-BSP", "error",
              "thread_size_key '" + in.thread_size_key +
              "' not present in central _hydraulic_ports.hpp BspG table");
    }

    if (gSpec) {
        if (in.drain_dia_mm > gSpec->thread_minor_mm) {
            r.add("DFM-BSP", "error",
                  "drain_dia must be ≤ BSPP thread_minor (" +
                  std::to_string(gSpec->thread_minor_mm) + " mm)");
        }
        if (in.spotface_dia_mm <= gSpec->thread_major_mm) {
            r.add("DFM-BSP", "error",
                  "spotface_dia must be > BSPP thread_major (" +
                  std::to_string(gSpec->thread_major_mm) + " mm)");
        }
    }

    if (in.spotface_depth_mm < 1.0) {
        r.add("DFM-SPOTFACE", "warning",
              "spotface_depth < 1 mm — plug seat may not seal");
    }

    if (in.plate_thickness_mm < 3.0) {
        r.add("DFM-MARINE-PLATE", "error",
              "plate_thickness_mm < 3.0 mm (marine hull plate minimum)");
    }

    if (gSpec && in.plate_thickness_mm <= in.spotface_depth_mm) {
        r.add("DFM-SPOTFACE", "error",
              "plate_thickness must exceed spotface_depth");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bilge_drain_threaded_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Dir adir = in.axis_dir;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;

    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 &&
          adir.Z() > 0)) {
        throw SkillError(
            "bilge_drain_threaded_compound: only axis_dir = +Z is supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ    = zMax;
    const double bottomZ = topZ - in.plate_thickness_mm;
    const double kOver   = 0.1;

    const auto* gSpec = hyd_ports::findBspG(in.thread_size_key);
    // cosmetic thread band: cylinder of thread_minor dia from spotface bottom
    // down by min(gSpec->depth_mm, plate_thickness - spotface_depth).
    const double threadBandDepth = std::min(
        gSpec->depth_mm,
        in.plate_thickness_mm - in.spotface_depth_mm);

    // ── A) drain through-bore ──────────────────────────────────────────
    const gp_Ax2 axBore(gp_Pnt(cx, cy, bottomZ - kOver), adir);
    const TopoDS_Shape boreTool = pr::cylinder(
        axBore, in.drain_dia_mm / 2.0,
        in.plate_thickness_mm + 2.0 * kOver);
    const TopoDS_Shape afterBore = pr::cut(wp.shape(), boreTool);

    // ── B) spotface counterbore at entry face ──────────────────────────
    const gp_Ax2 axSpot(gp_Pnt(cx, cy, topZ - in.spotface_depth_mm), adir);
    const TopoDS_Shape spotTool = pr::cylinder(
        axSpot, in.spotface_dia_mm / 2.0,
        in.spotface_depth_mm + kOver);
    const TopoDS_Shape afterSpot = pr::cut(afterBore, spotTool);

    // ── C) cosmetic thread band (annular slot at thread_minor in the bore wall)
    //      from below the spotface, depth = threadBandDepth.
    const double threadBandTopZ = topZ - in.spotface_depth_mm;
    const gp_Ax2 axThread(
        gp_Pnt(cx, cy, threadBandTopZ - threadBandDepth), adir);
    const TopoDS_Shape threadTool = pr::annularRing(
        axThread,
        gSpec->thread_minor_mm / 2.0,
        in.drain_dia_mm        / 2.0,
        threadBandDepth + kOver);
    const TopoDS_Shape newShape = pr::cut(afterSpot, threadTool);

    // ── signature ──────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",        cx },
        { "center_y_mm",        cy },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size_key",    in.thread_size_key },
        { "spotface_dia_mm",    in.spotface_dia_mm },
        { "spotface_depth_mm",  in.spotface_depth_mm },
        { "drain_dia_mm",       in.drain_dia_mm },
        { "plate_thickness_mm", in.plate_thickness_mm },
    };

    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "marine_feature_type", "bilge_drain_threaded" },
        { "subfeature_count",    3 },
        { "thread_size",         in.thread_size_key },
        { "thread_major_mm",     gSpec->thread_major_mm },
        { "thread_minor_mm",     gSpec->thread_minor_mm },
        { "thread_pitch_mm",     gSpec->pitch_mm },
        { "drain_dia_mm",        in.drain_dia_mm },
        { "spotface_dia_mm",     in.spotface_dia_mm },
        { "spotface_depth_mm",   in.spotface_depth_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "standard",            "ISO 228-1 BSPP" },
    };

    ToolingMeta tooling;
    tooling.tool_type = "drill;counterbore;thread_mill";
    tooling.tool_dia_mm = in.spotface_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count = 4;
    tooling.cutting_speed_sfm = 210.0;
    const double rDrain  = in.drain_dia_mm / 2.0;
    const double rSpot   = in.spotface_dia_mm / 2.0;
    const double rThr    = gSpec->thread_minor_mm / 2.0;
    const double vBore   = M_PI * rDrain * rDrain * in.plate_thickness_mm;
    const double vSpot   = M_PI * (rSpot * rSpot - rDrain * rDrain) *
                           in.spotface_depth_mm;
    const double vThread = M_PI * (rThr * rThr - rDrain * rDrain) *
                           threadBandDepth;
    tooling.stock_removed_mm3 = vBore + vSpot + vThread;
    tooling.est_cycle_time_s = 45.0;
    tooling.extra = {
        { "removed_volume_mm3", vBore + vSpot + vThread },
        { "standard",           "ISO 228-1 BSPP + marine bilge drain" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::bilge_drain_threaded_compound: thread={} drain={} faces {}→{}",
                  in.thread_size_key, in.drain_dia_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

struct CylRec {
    int    faceIdx = -1;
    double radius = 0.0;
    gp_Ax1 axis;
};

std::vector<CylRec> collectCylinders(const Workpiece& wp)
{
    std::vector<CylRec> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        out.push_back({ i, c.Radius(), c.Axis() });
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence = 1.0;
        rf.matched_geometry = { { "via", "metadata_replay" } };
        out.push_back(rf);
    }

    const auto cyls = collectCylinders(wp);
    if (cyls.size() < 2) return out;

    // Find a small bore and a coaxial larger spotface.
    for (const auto& bore : cyls) {
        if (std::abs(bore.axis.Direction().Z()) < 0.9) continue;
        for (const auto& spot : cyls) {
            if (spot.faceIdx == bore.faceIdx) continue;
            if (std::abs(spot.axis.Direction().Z()) < 0.9) continue;
            const double dx = spot.axis.Location().X() - bore.axis.Location().X();
            const double dy = spot.axis.Location().Y() - bore.axis.Location().Y();
            if (std::hypot(dx, dy) > 2.0) continue;
            if (spot.radius <= bore.radius * 1.2) continue;

            json recovered = {
                { "center_x_mm",      bore.axis.Location().X() },
                { "center_y_mm",      bore.axis.Location().Y() },
                { "axis_dir",         { 0.0, 0.0, 1.0 } },
                { "drain_dia_mm",     2.0 * bore.radius },
                { "spotface_dia_mm",  2.0 * spot.radius },
            };
            json matched = {
                { "bore_face_id",     bore.faceIdx },
                { "spotface_face_id", spot.faceIdx },
                { "via",              "geometric_spotface_threaded_drain" },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.8, matched });
            break;
        }
    }

    return out;
}

}  // namespace koocadcam::skill::bilge_drain_threaded_compound
