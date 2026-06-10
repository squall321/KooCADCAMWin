// @lat: [[engine/skills#deep_draw]]

#include "deep_draw.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Fillets.hpp"
#include "engine/primitives/Tools.hpp"

#include <Bnd_Box.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::deep_draw {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

double sheetThickness(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

// Slice-9 fix: after a deep_draw, the workpiece bbox grows by cup_depth so
// the naive sheetThickness() returns the WHOLE span, not the original sheet
// thickness.  This helper scans the result for the dominant pair of parallel
// horizontal planar faces (the sheet top + bottom) and reports their
// separation.  Falls back to the naive bbox dim if no such pair is found.
double sheetThicknessFromFaces(const Workpiece& wp)
{
    double bestArea = 0.0;
    double bestZ0   = 0.0;
    double bestZ1   = 0.0;
    bool   found    = false;
    const int n = wp.faceCount();
    // Collect horizontal-normal planar faces with their Z and area.
    struct HFace { double z; double area; };
    std::vector<HFace> hf;
    for (int i = 0; i < n; ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir nrm;
        try { nrm = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(nrm.Z()) < 0.95) continue;
        const double area = wp.faceArea(i);
        if (area <= 0.0) continue;
        const double z = wp.faceCenter(i).Z();
        hf.push_back({ z, area });
    }
    // Find pair of largest faces (one upward, one downward) with sheet-like
    // Z gap (1.0 – 5.0 mm typical sheet thickness).
    for (size_t i = 0; i < hf.size(); ++i) {
        for (size_t j = i + 1; j < hf.size(); ++j) {
            const double dz = std::abs(hf[i].z - hf[j].z);
            if (dz < 0.05 || dz > 6.0) continue;
            const double prod = hf[i].area * hf[j].area;
            if (prod > bestArea) {
                bestArea = prod;
                bestZ0   = hf[i].z;
                bestZ1   = hf[j].z;
                found    = true;
            }
        }
    }
    if (found) return std::abs(bestZ1 - bestZ0);
    return sheetThickness(wp);
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cup_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "deep_draw cup_dia_mm must be > 0");
    }
    if (in.cup_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "deep_draw cup_depth_mm must be > 0");
    }
    if (in.corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error", "deep_draw corner_r_mm must be >= 0");
    }

    const double tDetected = sheetThickness(wp);
    const double t = (in.sheet_thickness_mm > 0.0) ? in.sheet_thickness_mm : tDetected;
    if (tDetected > 5.0 && in.sheet_thickness_mm <= 0.0) {
        r.add("DFM-DD-SHEET", "error",
              "deep_draw: stock thickness " + std::to_string(tDetected) +
              " > 5 mm — not sheet metal");
    }

    if (in.cup_dia_mm > 0.0 && in.cup_depth_mm > 0.0) {
        const double ratio = in.cup_depth_mm / in.cup_dia_mm;
        if (ratio > 0.7) {
            r.add("DFM-DD-RATIO", "error",
                  "deep_draw drawing ratio " + std::to_string(ratio) +
                  " > 0.7 — single-pass deep-draw limit exceeded (redraw required)");
        }
    }

    if (in.corner_r_mm > 0.0 && t > 0.0 && in.corner_r_mm < 4.0 * t) {
        r.add("DFM-DD-CORNER", "error",
              "deep_draw corner_r " + std::to_string(in.corner_r_mm) +
              " mm < 4 × sheet_thickness (" + std::to_string(4.0 * t) +
              " mm) — corner tearing risk");
    }
    if (in.corner_r_mm <= 0.0 && t > 0.0) {
        r.add("DFM-DD-CORNER", "error",
              "deep_draw corner_r must be ≥ 4 × sheet_thickness (" +
              std::to_string(4.0 * t) + " mm)");
    }

    // Punch must fit on the sheet.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double R = in.cup_dia_mm / 2.0;
    if (in.cup_position_x_mm - R < xMin - 1e-6 ||
        in.cup_position_x_mm + R > xMax + 1e-6 ||
        in.cup_position_y_mm - R < yMin - 1e-6 ||
        in.cup_position_y_mm + R > yMax + 1e-6)
    {
        r.add("DFM-INPUT", "error",
              "deep_draw cup at (" + std::to_string(in.cup_position_x_mm) + "," +
              std::to_string(in.cup_position_y_mm) + ") r=" + std::to_string(R) +
              " falls off the sheet bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Strategy: cut a through-hole of cup_dia at (x, y) → fuse a hollow cup
// (cylindrical wall + bottom disc) that protrudes BELOW the sheet plane.
// The cup wall thickness equals the sheet thickness.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "deep_draw DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double tDetected = sheetThickness(wp);
    const double t = (in.sheet_thickness_mm > 0.0) ? in.sheet_thickness_mm : tDetected;

    const double sheetTopZ = zMax;
    const double sheetBotZ = zMin;
    const double cupR = in.cup_dia_mm / 2.0;
    const double innerR = std::max(0.001, cupR - t);

    // 1) Cut a through-hole of cup_dia through the sheet, so the cup wall
    //    can fuse cleanly into the opening.
    const gp_Ax2 holeAx(
        gp_Pnt(in.cup_position_x_mm, in.cup_position_y_mm, sheetBotZ - 0.5),
        gp::DZ());
    const TopoDS_Shape holeTool = pr::cylinder(holeAx, cupR, (sheetTopZ - sheetBotZ) + 1.0);
    TopoDS_Shape stage1;
    try {
        stage1 = pr::cut(wp.shape(), holeTool);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("deep_draw: through-hole cut failed: ") + ex.what());
    }

    // 2) Build the cup body: a hollow cylinder protruding BELOW the sheet plane.
    //    Outer cylinder of radius cupR, inner cylinder of radius innerR,
    //    height = cup_depth + sheet_thickness (so the lip overlaps the
    //    sheet's hole and merges cleanly during fuse).
    const double cupTotalHeight = in.cup_depth_mm + t;
    const gp_Ax2 cupAx(
        gp_Pnt(in.cup_position_x_mm, in.cup_position_y_mm,
               sheetBotZ - in.cup_depth_mm),
        gp::DZ());
    TopoDS_Shape cupOuter;
    TopoDS_Shape cupInner;
    try {
        cupOuter = pr::cylinder(cupAx, cupR, cupTotalHeight);
        // Inner cavity starts above the bottom by sheet_thickness (so the
        // bottom of the cup is a flat disc of thickness t).
        const gp_Ax2 innerAx(
            gp_Pnt(in.cup_position_x_mm, in.cup_position_y_mm,
                   sheetBotZ - in.cup_depth_mm + t),
            gp::DZ());
        cupInner = pr::cylinder(innerAx, innerR, cupTotalHeight);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("deep_draw: cup cyl build failed: ") + ex.what());
    }
    TopoDS_Shape cup;
    try {
        cup = pr::cut(cupOuter, cupInner);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("deep_draw: cup hollow cut failed: ") + ex.what());
    }

    // 3) Apply a fillet at the cup wall→bottom corner if corner_r > 0.
    //    Use predicate: edges whose mid-Z is near (sheetBotZ - cup_depth + t).
    if (in.corner_r_mm > 0.0) {
        const double cornerZ = sheetBotZ - in.cup_depth_mm + t;
        // Conservatively skip fillet on degenerate corner_r vs innerR cases.
        const double safeR = std::min(in.corner_r_mm, innerR * 0.8);
        if (safeR > 1e-3) {
            try {
                cup = pr::filletEdges(cup, safeR, pr::edgesAtZ(cornerZ, 0.05));
            } catch (const Standard_Failure& ex) {
                spdlog::warn("deep_draw: corner fillet failed ({}); continuing without fillet",
                             ex.what());
            }
        }
    }

    // 4) Fuse the cup with the sheet (with through-hole).
    TopoDS_Shape result;
    try {
        result = pr::fuse(stage1, cup);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("deep_draw: final fuse failed: ") + ex.what());
    }

    // Signature.
    json params = {
        { "cup_position_x_mm",   in.cup_position_x_mm },
        { "cup_position_y_mm",   in.cup_position_y_mm },
        { "cup_dia_mm",          in.cup_dia_mm },
        { "cup_depth_mm",        in.cup_depth_mm },
        { "corner_r_mm",         in.corner_r_mm },
        { "sheet_thickness_mm",  t },
    };
    const double drawingRatio = (in.cup_dia_mm > 0.0)
                                 ? in.cup_depth_mm / in.cup_dia_mm : 0.0;
    json pattern = {
        { "kind",                   kSkillId },
        { "cup_dia_mm",             in.cup_dia_mm },
        { "cup_depth_mm",           in.cup_depth_mm },
        { "sheet_thickness_mm",     t },
        { "drawing_ratio",          drawingRatio },
        { "cup_axis_dir",           { 0.0, 0.0, -1.0 } },
        { "corner_r_mm",            in.corner_r_mm },
        { "wall_thickness_mm",      t },
        { "cylindrical_wall",       true },
        { "circular_bottom_present", true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "deep_draw_punch_die";
    tooling.tool_dia_mm       = in.cup_dia_mm;
    tooling.tool_length_mm    = in.cup_depth_mm + 5.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // forming, not removal
    tooling.est_cycle_time_s  = 1.5 + in.cup_depth_mm * 0.05;
    tooling.extra["machining_constraint"] =
        "Deep draw — punch + matched die; blank-holder required to avoid "
        "wrinkling; lubrication needed for steel sheet";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::deep_draw applied: dia={} depth={} faces {}→{}",
                  in.cup_dia_mm, in.cup_depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Look for a vertical-axis cylindrical face whose axial extent is >
// sheet_thickness (i.e. it's a true cup, not just a sheet-thickness hole).
// The cup axis is vertical, the bottom is a small planar disc, the top
// (rim) sits at the sheet plane.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    // Slice-9: use face-pair detector — deep_draw makes the bbox grow by
    // cup_depth so plain bbox-min would falsely return the cup-included span.
    const double t = sheetThicknessFromFaces(wp);
    if (t <= 0.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // First pass: collect all vertical-axis cylindrical face candidates with
    // span > 1.5 × sheet_thickness (= clear cup-like protrusion).
    struct Cand {
        int    fIdx;
        double radius;
        gp_Pnt axisLoc;
        gp_Dir axisDir;
        double zLow;
        double zHigh;
    };
    std::vector<Cand> cands;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius   = cyl.Radius();
        const gp_Ax1  axis    = cyl.Axis();
        const gp_Dir  adir    = axis.Direction();
        if (std::abs(adir.Z()) < 0.9) continue;

        // Try circular edge bounds first (clean, robust on virgin cylinders).
        // Boolean fuses often replace the rim circles with non-circular
        // intersection curves, so fall back to the face's geometric Z bbox
        // when fewer than two circles are available.
        std::vector<double> zCircles;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            zCircles.push_back(crv.Circle().Location().Z());
        }
        double zLow  = 0.0;
        double zHigh = 0.0;
        if (zCircles.size() >= 2) {
            zLow  = *std::min_element(zCircles.begin(), zCircles.end());
            zHigh = *std::max_element(zCircles.begin(), zCircles.end());
        } else {
            Bnd_Box fb;
            BRepBndLib::Add(cylFace, fb, false);
            if (fb.IsVoid()) continue;
            double fxMin, fyMin, fzMin, fxMax, fyMax, fzMax;
            fb.Get(fxMin, fyMin, fzMin, fxMax, fyMax, fzMax);
            zLow  = fzMin;
            zHigh = fzMax;
        }
        const double span  = zHigh - zLow;
        // Relaxed span filter: only require span > sheet thickness (any cup
        // protrusion is at least t deep into the sheet).  Originally 1.5*t
        // — that was too strict after Boolean fuse trimming.
        if (span <= t * 1.05) continue;

        cands.push_back({ fIdx, radius, axis.Location(), adir, zLow, zHigh });
    }

    // Group by axis XY (within 0.5 mm — XY-only because coaxial cylinders
    // can have gp_Cylinder reference points at different Z, e.g. the cup
    // outer constructed at the cup floor vs. the cup inner offset upward
    // by the sheet thickness).  Choose the LARGEST radius per group as
    // the cup outer wall (= reported cup_dia).
    constexpr double kAxisXYTolMm = 0.5;
    std::vector<bool> used(cands.size(), false);
    for (size_t i = 0; i < cands.size(); ++i) {
        if (used[i]) continue;
        Cand best = cands[i];
        used[i] = true;
        for (size_t j = i + 1; j < cands.size(); ++j) {
            if (used[j]) continue;
            if (std::hypot(best.axisLoc.X() - cands[j].axisLoc.X(),
                           best.axisLoc.Y() - cands[j].axisLoc.Y()) > kAxisXYTolMm)
                continue;
            used[j] = true;
            if (cands[j].radius > best.radius) best = cands[j];
        }
        const double span = best.zHigh - best.zLow;
        const double cupDepth = std::max(0.0, span - t);
        const double cupDia   = 2.0 * best.radius;
        const double drawingRatio = (cupDia > 0.0) ? (cupDepth / cupDia) : 0.0;
        if (drawingRatio <= 0.0) continue;

        json recovered = {
            { "cup_position_x_mm",    best.axisLoc.X() },
            { "cup_position_y_mm",    best.axisLoc.Y() },
            { "cup_dia_mm",           cupDia },
            { "cup_depth_mm",         cupDepth },
            { "corner_r_mm",          0.0 },
            { "sheet_thickness_mm",   t },
        };
        json matched = {
            { "cylindrical_face_id",  best.fIdx },
            { "axis_dir",             { best.axisDir.X(), best.axisDir.Y(), best.axisDir.Z() } },
            { "z_bottom",             best.zLow },
            { "z_top",                best.zHigh },
            { "drawing_ratio",        drawingRatio },
            { "sheet_thickness_mm",   t },
        };
        double confidence = 0.75;
        if (drawingRatio > 0.7) confidence -= 0.1;
        if (drawingRatio < 0.05) confidence -= 0.2;
        confidence = std::clamp(confidence, 0.1, 0.95);

        out.push_back(RecognizedFeature{ kSkillId, recovered, confidence, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::deep_draw
