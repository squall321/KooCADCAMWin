// @lat: [[engine/skills#spring_contact_clip]]

#include "spring_contact_clip.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::spring_contact_clip {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.clip_width_mm < 1.5) {
        r.add("DFM-UL486A", "error",
              "clip_width " + std::to_string(in.clip_width_mm) +
              " mm < 1.5 mm — sub-mm class needs EDM, not milling");
    }
    if (in.clip_length_mm <= 0.0 || in.clip_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "clip_length/depth must be > 0");
    }
    if (in.clip_depth_mm < 1.0) {
        r.add("DFM-002", "error",
              "clip_depth_mm < 1.0 mm — endmill cannot reach U bottom");
    }
    if (in.back_stop_depth_mm <= 0.0 ||
        in.back_stop_depth_mm >= in.clip_depth_mm) {
        r.add("DFM-CLIP-BACKSTOP", "error",
              "back_stop_depth must be in (0, clip_depth)");
    }
    if (in.back_stop_width_mm >= in.clip_width_mm) {
        r.add("DFM-CLIP-BACKSTOP", "error",
              "back_stop_width must be < clip_width");
    }
    if (in.barb_slot_width_mm < 0.5) {
        r.add("DFM-002", "error",
              "barb_slot_width " + std::to_string(in.barb_slot_width_mm) +
              " mm < 0.5 mm");
    }
    if (in.barb_slot_height_mm <= 0.0 ||
        in.barb_slot_depth_mm  <= 0.0) {
        r.add("DFM-INPUT", "error", "barb_slot dimensions must be > 0");
    }
    if (in.barb_slot_z_offset_mm < 0.0 ||
        in.barb_slot_z_offset_mm + in.barb_slot_height_mm > in.clip_depth_mm) {
        r.add("DFM-CLIP-BARB", "error",
              "barb_slot must fit between entry face and clip floor");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spring_contact_clip DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("spring_contact_clip: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("spring_contact_clip: only axis_dir = -Z supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zEntry = zMax;
    const double kOverhang = 0.05;

    // Local axes: xLoc = orientation, yLoc = perpendicular.
    const double th = in.orientation_deg * M_PI / 180.0;
    const gp_Dir xLoc(std::cos(th), std::sin(th), 0.0);
    const gp_Dir yLoc(-std::sin(th), std::cos(th), 0.0);

    // 1) U-pocket (the main rectangular pocket).
    //    We use a box centered at (position, zEntry - clip_depth/2), spanning
    //    clip_length × clip_width × clip_depth.
    const gp_Pnt uOrigin(
        in.position_x_mm
            - in.clip_length_mm / 2.0 * xLoc.X()
            - in.clip_width_mm  / 2.0 * yLoc.X(),
        in.position_y_mm
            - in.clip_length_mm / 2.0 * xLoc.Y()
            - in.clip_width_mm  / 2.0 * yLoc.Y(),
        zEntry - in.clip_depth_mm);
    const gp_Ax2 uAx(uOrigin, gp_Dir(0, 0, 1), xLoc);
    const TopoDS_Shape uPocket = pr::box(
        uAx, in.clip_length_mm, in.clip_width_mm,
        in.clip_depth_mm + kOverhang);

    // 2) Back-stop pocket: a smaller box at the +x end of the U-pocket.
    //    Centered on the +xLoc end, at the bottom (z = zEntry - clip_depth -
    //    back_stop_depth, extending downward by back_stop_depth).
    const gp_Pnt bsCenter(
        in.position_x_mm + (in.clip_length_mm / 2.0) * xLoc.X(),
        in.position_y_mm + (in.clip_length_mm / 2.0) * xLoc.Y(),
        zEntry - in.clip_depth_mm);
    const gp_Pnt bsOrigin(
        bsCenter.X()
            - (in.clip_length_mm * 0.2) / 2.0 * xLoc.X()
            - in.back_stop_width_mm / 2.0 * yLoc.X(),
        bsCenter.Y()
            - (in.clip_length_mm * 0.2) / 2.0 * xLoc.Y()
            - in.back_stop_width_mm / 2.0 * yLoc.Y(),
        bsCenter.Z() - in.back_stop_depth_mm);
    const gp_Ax2 bsAx(bsOrigin, gp_Dir(0, 0, 1), xLoc);
    const TopoDS_Shape backStop = pr::box(
        bsAx, in.clip_length_mm * 0.2, in.back_stop_width_mm,
        in.back_stop_depth_mm + kOverhang);

    // 3) Barb-slot — a side window cut into one of the long walls of the
    //    U-pocket.  Lives on the +yLoc wall (positive perpendicular side).
    //    Extends BEYOND that wall by barb_slot_depth_mm.
    const double wallY = in.clip_width_mm / 2.0;
    const gp_Pnt barbOrigin(
        in.position_x_mm
            - in.barb_slot_width_mm / 2.0 * xLoc.X()
            + wallY * yLoc.X(),
        in.position_y_mm
            - in.barb_slot_width_mm / 2.0 * xLoc.Y()
            + wallY * yLoc.Y(),
        zEntry - in.barb_slot_z_offset_mm - in.barb_slot_height_mm);
    const gp_Ax2 barbAx(barbOrigin, gp_Dir(0, 0, 1), xLoc);
    const TopoDS_Shape barbSlot = pr::box(
        barbAx, in.barb_slot_width_mm, in.barb_slot_depth_mm,
        in.barb_slot_height_mm);

    // Fuse cutters → one cut.
    TopoDS_Shape cutter = pr::fuse(uPocket, backStop);
    cutter = pr::fuse(cutter, barbSlot);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",         *entryId },
        { "position_x_mm",         in.position_x_mm },
        { "position_y_mm",         in.position_y_mm },
        { "axis_dir",              { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "orientation_deg",       in.orientation_deg },
        { "clip_length_mm",        in.clip_length_mm },
        { "clip_width_mm",         in.clip_width_mm },
        { "clip_depth_mm",         in.clip_depth_mm },
        { "back_stop_width_mm",    in.back_stop_width_mm },
        { "back_stop_depth_mm",    in.back_stop_depth_mm },
        { "barb_slot_width_mm",    in.barb_slot_width_mm },
        { "barb_slot_height_mm",   in.barb_slot_height_mm },
        { "barb_slot_depth_mm",    in.barb_slot_depth_mm },
        { "barb_slot_z_offset_mm", in.barb_slot_z_offset_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "subfeature_count", 3 },
        { "u_pocket_extent",  { in.clip_length_mm, in.clip_width_mm, in.clip_depth_mm } },
        { "back_stop_extent", { in.back_stop_width_mm, in.back_stop_depth_mm } },
        { "barb_slot_extent", { in.barb_slot_width_mm, in.barb_slot_height_mm, in.barb_slot_depth_mm } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;side_mill";
    tooling.tool_dia_mm       = std::min(in.clip_width_mm, in.barb_slot_width_mm);
    tooling.tool_length_mm    = in.clip_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 =
        in.clip_length_mm * in.clip_width_mm * in.clip_depth_mm
      + (in.clip_length_mm * 0.2) * in.back_stop_width_mm * in.back_stop_depth_mm
      + in.barb_slot_width_mm * in.barb_slot_height_mm * in.barb_slot_depth_mm;
    tooling.est_cycle_time_s = std::max(3.0,
        in.clip_length_mm * in.clip_depth_mm * 0.03 + 2.0);
    tooling.extra = {
        { "standard",       "UL 486A (terminal connector mechanical retention)" },
        { "operation_note", "Mill U-pocket, plunge back-stop, side-mill barb window" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::spring_contact_clip applied: {}x{}x{} faces {}->{}",
                  in.clip_length_mm, in.clip_width_mm, in.clip_depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay (high confidence).
    for (const auto& sig : wp.features()) {
        if (sig.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = sig.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }

    // Geometric heuristic (PRIMARY): three pockets of distinct depth.
    // Find planar floor faces (normal +Z) below the workpiece top, count
    // their depth bands.  Need at least 3 distinct depths → very likely
    // this compound.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    std::vector<double> floorZs;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFacePlanar(fIdx)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(fIdx); } catch (...) { continue; }
        if (std::abs(n.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt c = wp.faceCenter(fIdx);
        if (std::abs(c.Z() - zMax) < 1e-3) continue;
        floorZs.push_back(c.Z());
    }
    std::sort(floorZs.begin(), floorZs.end());
    // Count distinct levels.
    std::vector<double> levels;
    for (double z : floorZs) {
        if (levels.empty() || std::abs(z - levels.back()) > 0.3)
            levels.push_back(z);
    }
    if (levels.size() < 2) return out;

    // Approximate clip_depth = zMax - shallowest_pocket_floor.
    const double clipDepth = zMax - levels.back();
    const double bsDepth   = levels.size() >= 2 ? (levels.back() - levels.front()) : 0.0;

    json recovered = {
        { "position_x_mm",      (xMin + xMax) / 2.0 },
        { "position_y_mm",      (yMin + yMax) / 2.0 },
        { "axis_dir",           { 0.0, 0.0, -1.0 } },
        { "clip_depth_mm",      clipDepth },
        { "back_stop_depth_mm", bsDepth },
    };
    RecognizedFeature r;
    r.skill_id         = kSkillId;
    r.recovered_params = recovered;
    r.confidence       = 0.6;
    r.matched_geometry = { { "floor_levels", static_cast<int>(levels.size()) } };
    out.push_back(r);
    return out;
}

}  // namespace koocadcam::skill::spring_contact_clip
