// @lat: [[engine/skills#laser_weld]]

#include "laser_weld.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::laser_weld {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.weld_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "laser_weld weld_width_mm must be > 0");
    }
    if (in.weld_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "laser_weld weld_depth_mm must be > 0");
    }
    if (in.weld_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "laser_weld weld_height_mm must be > 0");
    }
    if (in.weld_width_mm > 0.0 && in.weld_width_mm < 0.1) {
        r.add("DFM-LASER-WIDTH", "error",
              "laser_weld weld_width_mm " + std::to_string(in.weld_width_mm) +
              " < 0.1 mm (sub-resolution for industrial laser optics)");
    }
    if (in.weld_width_mm > 1.5) {
        r.add("DFM-LASER-WIDTH", "error",
              "laser_weld weld_width_mm " + std::to_string(in.weld_width_mm) +
              " > 1.5 mm (use MIG or hybrid laser-arc for wider beads)");
    }
    if (in.weld_width_mm > 0.0 && in.weld_depth_mm > 0.0) {
        const double aspect = in.weld_depth_mm / in.weld_width_mm;
        if (aspect < 3.0) {
            r.add("DFM-LASER-ASPECT", "error",
                  "laser_weld depth/width " + std::to_string(aspect) +
                  " < 3.0 — this is a conduction weld, not a keyhole; "
                  "increase laser_power_w or reduce weld_width_mm");
        }
    }
    const double seamLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                      in.end_y_mm - in.start_y_mm);
    if (seamLen <= 1e-6) {
        r.add("DFM-INPUT", "error", "laser_weld start and end must differ");
    }
    if (in.weld_width_mm > 0.0 && seamLen > 0.0 && seamLen <= in.weld_width_mm) {
        r.add("DFM-LASER-LEN", "warning",
              "laser_weld length " + std::to_string(seamLen) +
              " ≤ weld_width " + std::to_string(in.weld_width_mm) +
              " — consider a single laser spot");
    }
    if (in.laser_power_w <= 0.0) {
        r.add("DFM-LASER-PWR", "warning",
              "laser_weld laser_power_w must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "laser_weld DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("laser_weld: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("laser_weld: entry_face must be planar");

    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    // We model only the visible surface cap.  The keyhole depth lives in
    // metadata (weld_depth_mm) — the material below is re-melted in place,
    // not topologically removed or added.  Visible cap height is small (~0.1
    // weld_width by default).
    const double radius = in.weld_width_mm / 2.0;
    const double kSink  = in.weld_height_mm * 0.01;
    const double beadH  = in.weld_height_mm + kSink;
    const double baseZ  = faceCtr.Z();

    const gp_Pnt sBase(in.start_x_mm - outNorm.X() * kSink,
                       in.start_y_mm - outNorm.Y() * kSink,
                       baseZ          - outNorm.Z() * kSink);
    const gp_Pnt eBase(in.end_x_mm   - outNorm.X() * kSink,
                       in.end_y_mm   - outNorm.Y() * kSink,
                       baseZ          - outNorm.Z() * kSink);

    const gp_Ax2 axStart(sBase, outNorm);
    const gp_Ax2 axEnd  (eBase, outNorm);
    const TopoDS_Shape cylStart = pr::cylinder(axStart, radius, beadH);
    const TopoDS_Shape cylEnd   = pr::cylinder(axEnd,   radius, beadH);

    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm,
                       in.end_y_mm - in.start_y_mm,
                       0.0);
    const double seamLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / seamLen, dir2D.Y() / seamLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    const gp_Pnt boxOrigin(
        in.start_x_mm - radius * yLoc.X() - outNorm.X() * kSink,
        in.start_y_mm - radius * yLoc.Y() - outNorm.Y() * kSink,
        baseZ          - outNorm.Z() * kSink);
    const gp_Ax2 boxAx(boxOrigin, outNorm, xLoc);
    const TopoDS_Shape boxConn = pr::box(boxAx,
                                         seamLen,
                                         in.weld_width_mm,
                                         beadH);

    const TopoDS_Shape bead = pr::fuseMany(cylStart, { cylEnd, boxConn });
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), bead);

    const double aspect = in.weld_depth_mm / in.weld_width_mm;

    json params = {
        { "entry_face_id",       *entryId },
        { "start_x_mm",          in.start_x_mm },
        { "start_y_mm",          in.start_y_mm },
        { "end_x_mm",            in.end_x_mm },
        { "end_y_mm",            in.end_y_mm },
        { "weld_width_mm",       in.weld_width_mm },
        { "weld_depth_mm",       in.weld_depth_mm },
        { "weld_height_mm",      in.weld_height_mm },
        { "material",            in.material },
        { "laser_power_w",       in.laser_power_w },
        { "entry_face_normal",   { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "end_cylinder_count",      2 },
        { "long_wall_planar_count",  2 },
        { "top_planar_face",         true },
        { "weld_width_mm",           in.weld_width_mm },
        { "weld_depth_mm",           in.weld_depth_mm },
        { "weld_height_mm",          in.weld_height_mm },
        { "seam_length_mm",          seamLen },
        { "depth_to_width",          aspect },
        { "keyhole",                 aspect >= 3.0 },
        { "additive_cap",            true },
        // Sub-surface fusion zone — declared but not modeled in geometry.
        { "subsurface_fusion_modeled", false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "laser_head";
    tooling.tool_dia_mm       = in.weld_width_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "fiber_laser";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // The "deposited" volume is just the surface cap (the keyhole displaces
    // material laterally rather than removing or adding).  Record cap volume
    // as additive (negative) so accounting matches other welds.
    const double capVol =
        (M_PI * radius * radius + seamLen * in.weld_width_mm) * in.weld_height_mm;
    tooling.stock_removed_mm3 = -capVol;
    // Laser travel speed ~ 80 mm/s — fastest of the welds modelled here.
    tooling.est_cycle_time_s  = std::max(0.5, seamLen / 80.0);
    tooling.extra = {
        { "process",            "laser_keyhole" },
        { "material",           in.material },
        { "laser_power_w",      in.laser_power_w },
        { "weld_depth_mm",      in.weld_depth_mm },
        { "depth_to_width",     aspect },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::laser_weld applied: len={} w={} depth={} aspect={} faces {}→{}",
                  seamLen, in.weld_width_mm, in.weld_depth_mm, aspect,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometrically a laser weld looks like a very thin seam_weld.  We rely on
// metadata to confirm; an anonymous workpiece returns no candidates.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::laser_weld
