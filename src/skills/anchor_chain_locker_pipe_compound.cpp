// @lat: [[engine/skills#anchor_chain_locker_pipe_compound]]

#include "anchor_chain_locker_pipe_compound.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::anchor_chain_locker_pipe_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pipe_outer_dia_mm <= 0.0 || in.pipe_wall_thickness_mm <= 0.0 ||
        in.pipe_length_mm <= 0.0 || in.grating_ring_dia_mm <= 0.0 ||
        in.grating_ring_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "anchor_chain_locker_pipe_compound: dimensions must be > 0");
    }

    if (in.pipe_wall_thickness_mm < 3.0) {
        r.add("DFM-MARINE-PLATE", "error",
              "pipe_wall_thickness_mm < 3.0 mm (marine minimum)");
    }

    if (in.grating_ring_dia_mm <= in.pipe_outer_dia_mm) {
        r.add("DFM-GRATING", "error",
              "grating_ring_dia must be > pipe_outer_dia");
    }

    if (in.grating_ring_position_z_mm <= 0.0 ||
        in.grating_ring_position_z_mm >= in.pipe_length_mm) {
        r.add("DFM-GRATING", "error",
              "grating_ring_position_z must be within (0, pipe_length)");
    }

    if (in.grating_ring_depth_mm >= in.pipe_length_mm * 0.5) {
        r.add("DFM-GRATING", "error",
              "grating_ring_depth must be < pipe_length / 2");
    }

    // Inner bore must be > 0.
    const double innerBoreDia =
        in.pipe_outer_dia_mm - 2.0 * in.pipe_wall_thickness_mm;
    if (innerBoreDia <= 0.0) {
        r.add("DFM-PIPE-WALL", "error",
              "pipe_wall_thickness too large — inner bore would be ≤ 0");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "anchor_chain_locker_pipe_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Dir adir = in.axis_dir;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;

    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 &&
          adir.Z() > 0)) {
        throw SkillError(
            "anchor_chain_locker_pipe_compound: only axis_dir = +Z is supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ    = zMax;
    const double bottomZ = topZ - in.pipe_length_mm;
    const double kOver   = 0.1;

    // Bore is the pipe through-passage (inner dia = pipe OD - 2·wall).
    const double boreR = (in.pipe_outer_dia_mm * 0.5) - in.pipe_wall_thickness_mm;
    // For a chain-locker, what gets cut through the bulkhead is the OUTSIDE
    // pipe dia (clearance for the pipe wall) — we model the pipe seat as
    // pipe_outer_dia / 2.  Use the inner bore as the through-passage and a
    // wider annular pipe-wall recess from above.  To keep this simple, we
    // model the cut as pipe_outer_dia through the bulkhead, then a wider
    // annular recess at the grating position.

    // ── A) pipe through-bore (use OUTER dia — pipe passes through wall) ──
    const gp_Ax2 axBore(gp_Pnt(cx, cy, bottomZ - kOver), adir);
    const TopoDS_Shape boreTool = pr::cylinder(
        axBore, in.pipe_outer_dia_mm / 2.0,
        in.pipe_length_mm + 2.0 * kOver);
    const TopoDS_Shape afterBore = pr::cut(wp.shape(), boreTool);

    // ── B) annular grating support ring recess at grating_ring_position_z
    //      from the bottom of the pipe.  Z absolute = bottomZ + position.
    const double ringZ = bottomZ + in.grating_ring_position_z_mm;
    const gp_Ax2 axRing(gp_Pnt(cx, cy, ringZ - in.grating_ring_depth_mm * 0.5),
                        adir);
    const TopoDS_Shape ringTool = pr::annularRing(
        axRing,
        in.grating_ring_dia_mm / 2.0,
        in.pipe_outer_dia_mm   / 2.0,
        in.grating_ring_depth_mm);
    const TopoDS_Shape newShape = pr::cut(afterBore, ringTool);

    // ── signature ──────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",                cx },
        { "center_y_mm",                cy },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "pipe_outer_dia_mm",          in.pipe_outer_dia_mm },
        { "pipe_wall_thickness_mm",     in.pipe_wall_thickness_mm },
        { "pipe_length_mm",             in.pipe_length_mm },
        { "grating_ring_position_z_mm", in.grating_ring_position_z_mm },
        { "grating_ring_dia_mm",        in.grating_ring_dia_mm },
        { "grating_ring_depth_mm",      in.grating_ring_depth_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "marine_feature_type",        "chain_locker_pipe" },
        { "subfeature_count",           2 },
        { "pipe_outer_dia_mm",          in.pipe_outer_dia_mm },
        { "pipe_inner_bore_dia_mm",     2.0 * boreR },
        { "grating_ring_dia_mm",        in.grating_ring_dia_mm },
        { "grating_ring_position_z_mm", in.grating_ring_position_z_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "standard",                   "Lloyd's Register / IMO marine practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type = "drill;groove_cutter";
    tooling.tool_dia_mm = in.grating_ring_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count = 4;
    tooling.cutting_speed_sfm = 200.0;
    const double rPipe  = in.pipe_outer_dia_mm / 2.0;
    const double rRing  = in.grating_ring_dia_mm / 2.0;
    const double vBore  = M_PI * rPipe * rPipe * in.pipe_length_mm;
    const double vRing  = M_PI * (rRing * rRing - rPipe * rPipe) *
                          in.grating_ring_depth_mm;
    tooling.stock_removed_mm3 = vBore + vRing;
    tooling.est_cycle_time_s = 90.0;
    tooling.extra = {
        { "removed_volume_mm3", vBore + vRing },
        { "standard",           "chain locker pipe with grating recess" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::anchor_chain_locker_pipe_compound: pipe={} ring={} faces {}→{}",
                  in.pipe_outer_dia_mm, in.grating_ring_dia_mm,
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

    // Match a pipe bore with a coaxial larger grating recess.
    for (const auto& bore : cyls) {
        if (std::abs(bore.axis.Direction().Z()) < 0.9) continue;
        for (const auto& ring : cyls) {
            if (ring.faceIdx == bore.faceIdx) continue;
            if (std::abs(ring.axis.Direction().Z()) < 0.9) continue;
            const double dx = ring.axis.Location().X() - bore.axis.Location().X();
            const double dy = ring.axis.Location().Y() - bore.axis.Location().Y();
            if (std::hypot(dx, dy) > 2.0) continue;
            if (ring.radius <= bore.radius * 1.05) continue;

            json recovered = {
                { "center_x_mm",         bore.axis.Location().X() },
                { "center_y_mm",         bore.axis.Location().Y() },
                { "axis_dir",            { 0.0, 0.0, 1.0 } },
                { "pipe_outer_dia_mm",   2.0 * bore.radius },
                { "grating_ring_dia_mm", 2.0 * ring.radius },
            };
            json matched = {
                { "bore_face_id", bore.faceIdx },
                { "ring_face_id", ring.faceIdx },
                { "via",          "geometric_pipe_with_grating_ring" },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.8, matched });
            break;
        }
    }

    return out;
}

}  // namespace koocadcam::skill::anchor_chain_locker_pipe_compound
