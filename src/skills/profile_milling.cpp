// @lat: [[engine/skills#profile_milling]]

#include "profile_milling.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::profile_milling {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.offset_in_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "profile_milling offset_in must be > 0");
    }
    if (in.offset_in_mm > 0.0 && in.offset_in_mm < 0.4) {
        r.add("DFM-001", "error",
              "profile_milling offset_in " + std::to_string(in.offset_in_mm) +
              " mm < min 0.4 mm (wall thickness after profile)");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "profile_milling depth_mm must be > 0");
    }
    // Slice-1: axis_dir = -Z only.
    if (std::abs(in.axis_dir.X()) > 1e-6 ||
        std::abs(in.axis_dir.Y()) > 1e-6 ||
        in.axis_dir.Z() >= 0.0) {
        r.add("DFM-INPUT", "error",
              "slice-1: only axis_dir = -Z is supported");
    }

    // Offset must fit within bbox.
    if (in.offset_in_mm > 0.0) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double dx = xMax - xMin;
        const double dy = yMax - yMin;
        const double minDim = std::min(dx, dy);
        if (2.0 * in.offset_in_mm >= minDim - 1e-6) {
            r.add("DFM-INPUT", "error",
                  "2 × offset_in (" + std::to_string(2.0 * in.offset_in_mm) +
                  ") >= min bbox extent (" + std::to_string(minDim) +
                  ") — inset would collapse");
        }
        // Depth must fit within bbox height.
        if (in.depth_mm > 0.0 && in.depth_mm > (zMax - zMin) + 1e-6) {
            r.add("DFM-INPUT", "error",
                  "depth_mm exceeds workpiece thickness (" +
                  std::to_string(zMax - zMin) + " mm)");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "profile_milling DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("profile_milling: entry_face datum unresolved");

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());

    const double kOverhang = 0.05;
    const double off       = in.offset_in_mm;

    // Outer cutter box: covers the bbox in XY (with a tiny overhang to avoid
    // boundary-grazing issues) and goes from top down by `depth_mm`.
    // Construct via gp_Ax2(origin, +Z, +X) so DX→X, DY→Y, DZ→Z.
    const double outerOverXY = 0.10;  // overhang each side
    const gp_Pnt outerOrigin(
        bb.xMin - outerOverXY,
        bb.yMin - outerOverXY,
        bb.zMax - in.depth_mm);
    const gp_Ax2 outerAx(outerOrigin, gp::DZ(), gp::DX());
    const TopoDS_Shape outerBox = pr::box(outerAx,
        bb.dx() + 2.0 * outerOverXY,
        bb.dy() + 2.0 * outerOverXY,
        in.depth_mm + kOverhang);

    // Inner subtractor box: the INSET footprint (= bbox shrunk by `off` on
    // each side).  Same Z extent but extended slightly DEEPER so the
    // Boolean subtraction "inner from outer" produces a clean annular shape.
    const gp_Pnt innerOrigin(
        bb.xMin + off,
        bb.yMin + off,
        bb.zMax - in.depth_mm - kOverhang);
    const gp_Ax2 innerAx(innerOrigin, gp::DZ(), gp::DX());
    const TopoDS_Shape innerBox = pr::box(innerAx,
        bb.dx() - 2.0 * off,
        bb.dy() - 2.0 * off,
        in.depth_mm + 3.0 * kOverhang);

    // Annular cutter = outerBox − innerBox.
    const TopoDS_Shape annular = pr::cut(outerBox, innerBox);

    // Apply to workpiece: workpiece − annular.
    const TopoDS_Shape newShape = pr::cut(wp.shape(), annular);

    // Signature.
    json params = {
        { "entry_face_id",  *entryId },
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "offset_in_mm",   in.offset_in_mm },
        { "depth_mm",       in.depth_mm },
        { "bbox_dx_mm",     bb.dx() },
        { "bbox_dy_mm",     bb.dy() },
    };
    json pattern = {
        { "kind",                          kSkillId },
        // 4 new vertical walls form the perimeter of the inset.
        { "new_planar_wall_count",         4 },
        // 1 new annular step face at the bottom of the shell.
        { "step_planar_face_present",      true },
        { "offset_in_mm",                  in.offset_in_mm },
        { "depth_mm",                      in.depth_mm },
        { "inset_extent_x_mm",             bb.dx() - 2.0 * in.offset_in_mm },
        { "inset_extent_y_mm",             bb.dy() - 2.0 * in.offset_in_mm },
        { "axis_dir",                      { in.axis_dir.X(), in.axis_dir.Y(),
                                             in.axis_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = 6.0;            // typical profile end mill
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 600.0;
    tooling.feed_per_tooth_mm = 0.040;
    // Stock removed = annulus area × depth.
    // Annulus area = outer_area − inset_area.
    const double outerArea = bb.dx() * bb.dy();
    const double insetArea = (bb.dx() - 2.0 * off) * (bb.dy() - 2.0 * off);
    tooling.stock_removed_mm3 = (outerArea - insetArea) * in.depth_mm;
    // Profile path length = 2 (dx + dy) → cycle ≈ path_len / (1000 mm/s).
    const double pathLen = 2.0 * (bb.dx() + bb.dy());
    tooling.est_cycle_time_s = std::max(2.0, pathLen / 100.0);
    tooling.extra = {
        { "removes", "outer_skin" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::profile_milling applied: offset={} depth={} bbox {}x{} faces {}→{}",
                  in.offset_in_mm, in.depth_mm, bb.dx(), bb.dy(),
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: the TOP entry-face is an inset rectangle, surrounded by a
// SHORTER annular planar face (the step at depth `depth_mm` below the
// top).  Topologically:
//   - find the largest planar face with normal = +Z (the island top)
//   - find a SECOND large planar face also with normal = +Z, at a lower Z
//     (the surrounding step / shell floor)
//   - offset_in_mm = (bbox.dx − island_dx) / 2
//   - depth_mm     = z(island_top) − z(step)
//
// This is a coarse heuristic — full recognition would inspect the 4 new
// vertical wall faces too.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());

    // Collect all +Z-facing planar faces with their (area, center).
    struct TopFace {
        int    faceIdx;
        double area;
        double z;
        double centerX;
        double centerY;
    };
    std::vector<TopFace> tops;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); }
        catch (...) { continue; }
        if (std::abs(n.Z() - 1.0) > 1e-2) continue;       // only +Z faces

        gp_Pnt c = wp.faceCenter(i);
        tops.push_back({ i, wp.faceArea(i), c.Z(), c.X(), c.Y() });
    }
    if (tops.size() < 2) return out;

    // Sort by Z descending → tops[0] = highest plane (island top), then a
    // strictly LOWER plane that is the step floor.
    std::sort(tops.begin(), tops.end(),
        [](const TopFace& a, const TopFace& b) { return a.z > b.z; });

    const TopFace& island = tops[0];
    // Find a second top-facing face whose Z is strictly lower than island
    // AND whose area is the LARGEST remaining (the shell floor).
    int stepIdx = -1;
    double bestStepArea = -1.0;
    for (size_t k = 1; k < tops.size(); ++k) {
        if (tops[k].z >= island.z - 1e-6) continue;        // must be lower
        if (tops[k].area > bestStepArea) {
            bestStepArea = tops[k].area;
            stepIdx = static_cast<int>(k);
        }
    }
    if (stepIdx < 0) return out;
    const TopFace& step = tops[stepIdx];

    // Estimate the island's XY extent from the bbox of its face.
    // (Simple approximation: face area = dx · dy assumed; we need the face's
    // own bbox.  Use the Workpiece's faceCenter + the global bbox — the
    // island sits centred on the workpiece.)
    //
    // For a true 2D bbox of the face we'd run BRepBndLib on the face;
    // for slice-1 we just compute the inset extent from the area ratio.
    // Island area = (bbox.dx − 2·off) × (bbox.dy − 2·off).
    // Solve for `off` from the area:
    //   A = (dx − 2·o) (dy − 2·o)
    //     = dx·dy − 2·o·(dx + dy) + 4·o²
    //   4 o² − 2(dx+dy)·o + (dx·dy − A) = 0
    const double A = island.area;
    const double dxBB = bb.dx();
    const double dyBB = bb.dy();
    const double aQ = 4.0;
    const double bQ = -2.0 * (dxBB + dyBB);
    const double cQ = dxBB * dyBB - A;
    const double disc = bQ * bQ - 4.0 * aQ * cQ;
    if (disc < 0.0) return out;
    const double sq = std::sqrt(disc);
    // Two roots — the physical one is the smaller positive root.
    const double off1 = (-bQ - sq) / (2.0 * aQ);
    const double off2 = (-bQ + sq) / (2.0 * aQ);
    double off = -1.0;
    if (off1 > 0.0 && off1 < std::min(dxBB, dyBB) / 2.0) off = off1;
    else if (off2 > 0.0 && off2 < std::min(dxBB, dyBB) / 2.0) off = off2;
    if (off < 0.0) return out;

    const double depth = island.z - step.z;
    if (depth <= 0.0) return out;

    // Confidence: how well does the island sit centered on the bbox?
    const double bboxCx = (bb.xMin + bb.xMax) / 2.0;
    const double bboxCy = (bb.yMin + bb.yMax) / 2.0;
    const double offsetErr = std::sqrt(
        (island.centerX - bboxCx) * (island.centerX - bboxCx) +
        (island.centerY - bboxCy) * (island.centerY - bboxCy));
    double conf = 0.80;
    if (offsetErr > 0.5) conf -= 0.20;
    conf = std::clamp(conf, 0.3, 0.85);

    json recovered = {
        { "entry_face_id",  island.faceIdx },
        { "axis_dir",       { 0.0, 0.0, -1.0 } },
        { "offset_in_mm",   off },
        { "depth_mm",       depth },
        { "bbox_dx_mm",     dxBB },
        { "bbox_dy_mm",     dyBB },
    };
    json matched = {
        { "island_face_id", island.faceIdx },
        { "step_face_id",   step.faceIdx },
        { "island_area",    A },
        { "step_z",         step.z },
        { "island_z",       island.z },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::profile_milling
