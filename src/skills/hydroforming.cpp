// @lat: [[engine/skills#hydroforming]]

#include "hydroforming.hpp"

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

namespace koocadcam::skill::hydroforming {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

double sheetThickness(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

// Slice-9 fix: after a hydroforming, the workpiece bbox grows by cup_depth
// so plain bbox-min returns the cup-included span.  Scan for the dominant
// pair of parallel horizontal planar faces (sheet top + bottom) instead.
double sheetThicknessFromFaces(const Workpiece& wp)
{
    struct HFace { double z; double area; };
    std::vector<HFace> hf;
    const int n = wp.faceCount();
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
    double bestProd = 0.0;
    double bestDz   = 0.0;
    bool   found    = false;
    for (size_t i = 0; i < hf.size(); ++i) {
        for (size_t j = i + 1; j < hf.size(); ++j) {
            const double dz = std::abs(hf[i].z - hf[j].z);
            if (dz < 0.05 || dz > 6.0) continue;
            const double prod = hf[i].area * hf[j].area;
            if (prod > bestProd) {
                bestProd = prod;
                bestDz   = dz;
                found    = true;
            }
        }
    }
    if (found) return bestDz;
    return sheetThickness(wp);
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cup_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "hydroforming cup_dia_mm must be > 0");
    }
    if (in.cup_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "hydroforming cup_depth_mm must be > 0");
    }
    if (in.corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error", "hydroforming corner_r_mm must be >= 0");
    }
    if (in.pressure_mpa < 50.0 || in.pressure_mpa > 600.0) {
        r.add("DFM-HF-PRESS", "error",
              "hydroforming pressure " + std::to_string(in.pressure_mpa) +
              " MPa out of [50, 600] — typical hydroform range");
    }

    const double tDetected = sheetThickness(wp);
    const double t = (in.sheet_thickness_mm > 0.0) ? in.sheet_thickness_mm : tDetected;
    if (tDetected > 5.0 && in.sheet_thickness_mm <= 0.0) {
        r.add("DFM-HF-SHEET", "error",
              "hydroforming: stock thickness " + std::to_string(tDetected) +
              " > 5 mm — not sheet metal");
    }

    if (in.cup_dia_mm > 0.0 && in.cup_depth_mm > 0.0) {
        const double ratio = in.cup_depth_mm / in.cup_dia_mm;
        if (ratio > 1.0) {
            r.add("DFM-HF-RATIO", "error",
                  "hydroforming drawing ratio " + std::to_string(ratio) +
                  " > 1.0 — single-pass hydroform limit");
        }
    }
    if (in.corner_r_mm > 0.0 && t > 0.0 && in.corner_r_mm < 2.0 * t) {
        r.add("DFM-HF-CORNER", "error",
              "hydroforming corner_r " + std::to_string(in.corner_r_mm) +
              " mm < 2 × sheet_thickness (" + std::to_string(2.0 * t) +
              " mm) — corner tearing risk");
    }
    if (in.corner_r_mm <= 0.0 && t > 0.0) {
        r.add("DFM-HF-CORNER", "error",
              "hydroforming corner_r must be ≥ 2 × sheet_thickness (" +
              std::to_string(2.0 * t) + " mm)");
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
              "hydroforming cup falls off the sheet bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Same shape result as deep_draw: a cup that protrudes BELOW the sheet
// plane.  Wall thickness assumed = sheet_thickness for slice-1 (the model
// could compute actual thinning from pressure if needed downstream).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hydroforming DFM failed:";
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
    // Hydroforming produces thinner walls than deep_draw under pressure.
    // Approximate post-form wall: t × (1 - pressure_mpa/2000) bounded below
    // by t × 0.5.  We use sheet_thickness as the wall thickness for slice-1
    // (the precise thinning model is a CAE concern, not a CAD concern).
    const double wallThickness = t;
    const double innerR = std::max(0.001, cupR - wallThickness);

    // 1) Cut a through-hole of cup_dia.
    const gp_Ax2 holeAx(
        gp_Pnt(in.cup_position_x_mm, in.cup_position_y_mm, sheetBotZ - 0.5),
        gp::DZ());
    const TopoDS_Shape holeTool = pr::cylinder(holeAx, cupR, (sheetTopZ - sheetBotZ) + 1.0);
    TopoDS_Shape stage1;
    try {
        stage1 = pr::cut(wp.shape(), holeTool);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("hydroforming: hole cut failed: ") + ex.what());
    }

    // 2) Build the cup body protruding BELOW the sheet.
    const double cupTotalHeight = in.cup_depth_mm + wallThickness;
    const gp_Ax2 cupAx(
        gp_Pnt(in.cup_position_x_mm, in.cup_position_y_mm,
               sheetBotZ - in.cup_depth_mm),
        gp::DZ());
    TopoDS_Shape cup;
    try {
        const TopoDS_Shape cupOuter =
            pr::cylinder(cupAx, cupR, cupTotalHeight);
        const gp_Ax2 innerAx(
            gp_Pnt(in.cup_position_x_mm, in.cup_position_y_mm,
                   sheetBotZ - in.cup_depth_mm + wallThickness),
            gp::DZ());
        const TopoDS_Shape cupInner = pr::cylinder(innerAx, innerR, cupTotalHeight);
        cup = pr::cut(cupOuter, cupInner);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("hydroforming: cup build failed: ") + ex.what());
    }

    if (in.corner_r_mm > 0.0) {
        const double cornerZ = sheetBotZ - in.cup_depth_mm + wallThickness;
        const double safeR = std::min(in.corner_r_mm, innerR * 0.8);
        if (safeR > 1e-3) {
            try {
                cup = pr::filletEdges(cup, safeR, pr::edgesAtZ(cornerZ, 0.05));
            } catch (const Standard_Failure& ex) {
                spdlog::warn("hydroforming: corner fillet failed ({}); continuing",
                             ex.what());
            }
        }
    }

    TopoDS_Shape result;
    try {
        result = pr::fuse(stage1, cup);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("hydroforming: final fuse failed: ") + ex.what());
    }

    // Signature.
    json params = {
        { "cup_position_x_mm",   in.cup_position_x_mm },
        { "cup_position_y_mm",   in.cup_position_y_mm },
        { "cup_dia_mm",          in.cup_dia_mm },
        { "cup_depth_mm",        in.cup_depth_mm },
        { "corner_r_mm",         in.corner_r_mm },
        { "sheet_thickness_mm",  t },
        { "pressure_mpa",        in.pressure_mpa },
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
        { "wall_thickness_mm",      wallThickness },
        { "pressure_mpa",           in.pressure_mpa },
        { "cylindrical_wall",       true },
        { "circular_bottom_present", true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "hydroform_die";
    tooling.tool_dia_mm       = in.cup_dia_mm;
    tooling.tool_length_mm    = in.cup_depth_mm + 5.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 5.0 + in.cup_depth_mm * 0.1;
    tooling.extra["machining_constraint"] =
        "Hydroforming — high-pressure fluid presses sheet against die; "
        "complex geometries achievable; uniform wall thinning";
    tooling.extra["pressure_mpa"] = in.pressure_mpa;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::hydroforming applied: dia={} depth={} p={} MPa",
                  in.cup_dia_mm, in.cup_depth_mm, in.pressure_mpa);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometrically the same as deep_draw — a cylindrical protrusion from the
// sheet.  We return candidates with LOWER baseline confidence than deep_draw
// unless the workpiece feature history shows a prior `hydroforming` entry.
// Downstream layers can choose between deep_draw and hydroforming based on
// the recovered confidence and any external context (process metadata).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    // Slice-9: use face-pair detector to recover sheet thickness — apply()
    // grew the bbox by cup_depth so bbox-min returns the cup span.
    const double t = sheetThicknessFromFaces(wp);
    if (t <= 0.0) return out;

    // History-aware boost: prior hydroforming?
    bool hasHistory = false;
    double historyPressure = 0.0;
    for (const auto& f : wp.features()) {
        if (f.skill_id == kSkillId) {
            hasHistory = true;
            try { historyPressure = f.pattern.value("pressure_mpa", 0.0); }
            catch (...) {}
            break;
        }
    }

    // Collect all vertical-axis cup-like candidates.
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

        // Try circular edge bounds first.  Boolean fuses can replace the
        // rim circles with non-circular intersection curves, so fall back
        // to the face's geometric Z bbox when fewer than two circles exist.
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
        // Relaxed span filter (originally 1.5*t — too strict after fuse).
        if (span <= t * 1.05) continue;

        cands.push_back({ fIdx, radius, axis.Location(), adir, zLow, zHigh });
    }

    // Group by axis XY (within 0.5 mm — XY-only because coaxial cylinders
    // can have gp_Cylinder reference points at different Z, e.g. the cup
    // outer at the cup floor vs. the cup inner offset upward by the wall
    // thickness).  Choose largest radius per group (cup outer wall).
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

        double confidence = 0.45;
        if (drawingRatio > 0.7) confidence += 0.2;
        if (hasHistory)         confidence += 0.3;
        confidence = std::clamp(confidence, 0.0, 0.95);

        json recovered = {
            { "cup_position_x_mm",    best.axisLoc.X() },
            { "cup_position_y_mm",    best.axisLoc.Y() },
            { "cup_dia_mm",           cupDia },
            { "cup_depth_mm",         cupDepth },
            { "corner_r_mm",          0.0 },
            { "sheet_thickness_mm",   t },
            { "pressure_mpa",         hasHistory ? historyPressure : 200.0 },
        };
        json matched = {
            { "cylindrical_face_id",  best.fIdx },
            { "z_top",                best.zHigh },
            { "z_bottom",             best.zLow },
            { "drawing_ratio",        drawingRatio },
            { "metadata_history",     hasHistory },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, confidence, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::hydroforming
