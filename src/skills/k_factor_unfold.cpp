// @lat: [[engine/skills#k_factor_unfold]]

#include "k_factor_unfold.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::k_factor_unfold {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double sheetThicknessOf(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.sheet_thickness_mm < 0.5 || in.sheet_thickness_mm > 6.0) {
        r.add("DFM-K-SHEET", "error",
              "k_factor_unfold: sheet_thickness_mm " +
              std::to_string(in.sheet_thickness_mm) +
              " outside 0.5–6.0 mm sheet-metal range");
    }
    if (in.k_factor < 0.3 || in.k_factor > 0.5) {
        r.add("DFM-K-FACTOR", "error",
              "k_factor_unfold: k_factor " +
              std::to_string(in.k_factor) +
              " outside 0.3–0.5 (neutral-axis position)");
    }
    const double absA = std::abs(in.bend_angle_deg);
    if (absA <= 0.0 || absA > 180.0) {
        r.add("DFM-K-ANGLE", "error",
              "k_factor_unfold: bend_angle_deg " +
              std::to_string(in.bend_angle_deg) +
              " outside (0, 180] range");
    }
    if (in.inner_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "k_factor_unfold: inner_radius_mm must be > 0");
    }
    const double dirMag = std::sqrt(
        in.bend_axis_dir[0] * in.bend_axis_dir[0] +
        in.bend_axis_dir[1] * in.bend_axis_dir[1] +
        in.bend_axis_dir[2] * in.bend_axis_dir[2]);
    if (dirMag < 1e-6) {
        r.add("DFM-INPUT", "error",
              "k_factor_unfold: bend_axis_dir is degenerate (zero vector)");
    }
    // Sanity vs current workpiece thickness.
    const double t = sheetThicknessOf(wp);
    if (t > 0.0 && std::abs(t - in.sheet_thickness_mm) > 0.5 * t) {
        r.add("DFM-K-MISMATCH", "warning",
              "k_factor_unfold: provided sheet_thickness disagrees with stock bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// 1. Split the workpiece at the bend axis into two halves.
// 2. Rotate the "rotated" half BACK to the flat plane by -bend_angle.
// 3. Translate it outward along the unfold direction by the developed-length
//    correction (BA − chord) so the unfolded blank length equals the
//    K-factor developed length.
// 4. Fuse halves into a single flat sheet.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "k_factor_unfold DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    // Build bend axis.
    gp_Vec dirV(in.bend_axis_dir[0], in.bend_axis_dir[1], in.bend_axis_dir[2]);
    dirV.Normalize();
    gp_Pnt axisOrigin(in.bend_axis_origin[0],
                      in.bend_axis_origin[1],
                      in.bend_axis_origin[2]);
    const gp_Ax1 bendAxis(axisOrigin, gp_Dir(dirV));

    // K-factor developed length: BA = α (R + K·t)
    const double angleRad = std::abs(in.bend_angle_deg) * M_PI / 180.0;
    const double devLen   = angleRad *
                            (in.inner_radius_mm + in.k_factor * in.sheet_thickness_mm);
    // Chord (straight distance between bend start and end on the neutral axis).
    const double rN     = in.inner_radius_mm + in.k_factor * in.sheet_thickness_mm;
    const double chord  = 2.0 * rN * std::sin(angleRad * 0.5);
    const double correction = devLen - chord;        // ≥ 0 for α > 0

    // Split the workpiece using a half-space box on each side of the axis.
    // Perpendicular direction in XY for splitting (axis is treated as a line
    // in the sheet plane; we cut along it).
    gp_Vec perpV;
    {
        gp_Vec zUp(0.0, 0.0, 1.0);
        perpV = dirV.Crossed(zUp);
        if (perpV.Magnitude() < 1e-6) {
            // axis is vertical — fall back to X.
            perpV = gp_Vec(1.0, 0.0, 0.0);
        } else {
            perpV.Normalize();
        }
    }

    // Build two half-space cutters (boxes) extending ± perpV from axis origin.
    const double Lbig = bboxDiag * 4.0;
    auto buildHalfCutter = [&](double sign) -> TopoDS_Shape
    {
        // box anchor: 2·Lbig along -dirV from axisOrigin, 0 along perp,
        // below bbox in Z.
        const gp_Pnt corner(
            axisOrigin.X() - dirV.X() * Lbig
                           + perpV.X() * (sign > 0.0 ? 0.0 : -Lbig),
            axisOrigin.Y() - dirV.Y() * Lbig
                           + perpV.Y() * (sign > 0.0 ? 0.0 : -Lbig),
            zMin - 1.0);
        const gp_Ax2 ax(corner, gp::DZ(), gp_Dir(dirV));
        return pr::box(ax, 2.0 * Lbig, Lbig, bboxDiag * 2.0 + 2.0);
    };
    const TopoDS_Shape rightCutter = buildHalfCutter(+1.0);
    const TopoDS_Shape leftCutter  = buildHalfCutter(-1.0);

    // H_fixed   : keep LEFT  (subtract right half)
    // H_rotated : keep RIGHT (subtract left half)
    TopoDS_Shape H_fixed;
    TopoDS_Shape H_rotated_raw;
    try {
        H_fixed       = pr::cut(wp.shape(), rightCutter);
        H_rotated_raw = pr::cut(wp.shape(), leftCutter);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("k_factor_unfold: split failed: ") + ex.what());
    }

    // 2. Apply INVERSE rotation to H_rotated to bring it flat again.
    gp_Trsf rot;
    rot.SetRotation(bendAxis, -in.bend_angle_deg * M_PI / 180.0);
    BRepBuilderAPI_Transform xfRot(H_rotated_raw, rot, /*Copy=*/false);
    if (!xfRot.IsDone())
        throw SkillError("k_factor_unfold: unfold rotation failed");
    TopoDS_Shape H_rotated = xfRot.Shape();

    // 3. Translate the now-flat rotated half outward to insert the
    //    developed-length correction (avoids material overlap & preserves
    //    neutral-axis area).
    gp_Trsf trs;
    // Unfold direction in XY: perpendicular to bend axis, into the H_rotated
    // half-space (which we chose to be the +perp side).
    trs.SetTranslation(gp_Vec(perpV.X() * correction,
                              perpV.Y() * correction,
                              perpV.Z() * correction));
    BRepBuilderAPI_Transform xfTrs(H_rotated, trs, /*Copy=*/false);
    if (!xfTrs.IsDone())
        throw SkillError("k_factor_unfold: translation failed");
    H_rotated = xfTrs.Shape();

    // 4. Fuse.
    TopoDS_Shape result;
    try {
        result = pr::fuse(H_fixed, H_rotated);
    } catch (const Standard_Failure& ex) {
        spdlog::warn("k_factor_unfold: fuse failed ({}); keeping rotated half", ex.what());
        result = H_rotated;
    }

    // Signature.
    json params = {
        { "bend_axis_origin",    { in.bend_axis_origin[0],
                                   in.bend_axis_origin[1],
                                   in.bend_axis_origin[2] } },
        { "bend_axis_dir",       { dirV.X(), dirV.Y(), dirV.Z() } },
        { "bend_angle_deg",      in.bend_angle_deg },
        { "sheet_thickness_mm",  in.sheet_thickness_mm },
        { "k_factor",            in.k_factor },
        { "inner_radius_mm",     in.inner_radius_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "sheet_feature_type",    "unfold" },
        { "developed_length_mm",   devLen },
        { "neutral_radius_mm",     rN },
        { "chord_mm",              chord },
        { "correction_mm",         correction },
        { "sheet_thickness_mm",    in.sheet_thickness_mm },
        { "k_factor",              in.k_factor },
        { "bend_angle_deg",        in.bend_angle_deg },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "press_brake_unfold";
    tooling.tool_dia_mm       = in.inner_radius_mm * 2.0;
    tooling.tool_length_mm    = bboxDiag;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;            // CAD-only operation
    tooling.extra["machining_constraint"] =
        "Unfold (CAD-only) — produces the flat developed blank using "
        "K-factor neutral-axis position; downstream press-brake recreates bend";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::k_factor_unfold applied: α={} K={} R={} BA={} faces {}→{}",
                  in.bend_angle_deg, in.k_factor,
                  in.inner_radius_mm, devLen,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// An unfolded blank has:
//   - NO horizontal-axis cylindrical bend faces (those are the hallmark of a
//     bent sheet).
//   - One LARGE +Z planar face dominating the workpiece.
// We emit a candidate when these conditions hold and the bbox is plate-thin.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 6.0) return out;

    // Look for any horizontal-axis cylindrical face — if any are present,
    // this is still bent, not unfolded.
    int horizontalCylinders = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z()) < 0.5) ++horizontalCylinders;
    }
    if (horizontalCylinders > 0) return out;

    // Find largest +Z planar face.
    double largestArea = -1.0;
    int    largestId   = -1;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > largestArea) {
            largestArea = a;
            largestId   = i;
        }
    }
    if (largestId < 0) return out;

    json recovered = {
        { "sheet_thickness_mm", t },
        { "k_factor",           0.42 },           // assumed (cannot recover)
        { "inner_radius_mm",    1.0 },
        { "bend_angle_deg",     0.0 },             // already unfolded
    };
    json matched = {
        { "flat_face_id", largestId },
        { "flat_area",    largestArea },
        { "is_flat_blank", true },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.60, matched });
    return out;
}

}  // namespace koocadcam::skill::k_factor_unfold
