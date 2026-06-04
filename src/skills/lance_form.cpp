// @lat: [[engine/skills#lance_form]]

#include "lance_form.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::lance_form {

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

    const double t = sheetThicknessOf(wp);
    if (t < 0.5 || t > 6.0) {
        r.add("DFM-L-SHEET", "error",
              "lance_form: stock thickness " + std::to_string(t) +
              " mm outside 0.5–6.0 mm sheet-metal range");
    }
    const double dx = in.slit_end_xy[0] - in.slit_start_xy[0];
    const double dy = in.slit_end_xy[1] - in.slit_start_xy[1];
    const double slitLen = std::hypot(dx, dy);
    if (t > 0.0 && slitLen <= 5.0 * t) {
        r.add("DFM-L-SLIT-LEN", "error",
              "lance_form: slit length " + std::to_string(slitLen) +
              " mm must exceed 5 × thickness (" + std::to_string(5.0 * t) + ")");
    }
    if (t > 0.0 && in.form_height_mm < t) {
        r.add("DFM-L-HEIGHT", "error",
              "lance_form: form_height_mm " + std::to_string(in.form_height_mm) +
              " mm < thickness — not a formed tab");
    }
    if (t > 0.0 && in.form_height_mm > 4.0 * t) {
        r.add("DFM-L-HEIGHT", "error",
              "lance_form: form_height_mm " + std::to_string(in.form_height_mm) +
              " mm > 4 × thickness — risk of tear at slit root");
    }
    if (t > 0.0 && in.form_width_mm < 2.0 * t) {
        r.add("DFM-L-WIDTH", "error",
              "lance_form: form_width_mm " + std::to_string(in.form_width_mm) +
              " mm < 2 × thickness");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lance_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    (void)xMin; (void)yMin; (void)xMax; (void)yMax;
    const double t = sheetThicknessOf(wp);

    // Slit axis = slit_start → slit_end at sheet top (z = zMax).
    const gp_Pnt pA(in.slit_start_xy[0], in.slit_start_xy[1], zMin);
    const gp_Pnt pB(in.slit_end_xy[0],   in.slit_end_xy[1],   zMin);
    gp_Vec dirV(pA, pB);
    const double slitLen = dirV.Magnitude();
    dirV.Normalize();
    const gp_Vec perp(-dirV.Y(), dirV.X(), 0.0);

    // 1. Cut: narrow slit through the sheet along slit axis.
    //    Width of slit = thickness * 0.2 (typical kerf).
    const double slitKerf = std::max(t * 0.2, 0.1);
    // Build a thin box centred on slit axis, full sheet thickness.
    const gp_Pnt slitOrigin(
        pA.X() - perp.X() * (slitKerf * 0.5),
        pA.Y() - perp.Y() * (slitKerf * 0.5),
        zMin - 0.05);
    // Frame: mainZ = +Z, xDir = perp (dx = slitKerf), Y = dirV (dy = slitLen),
    //   so dz = sheet thickness + overhang.
    const gp_Ax2 slitAx(slitOrigin, gp::DZ(), gp_Dir(perp));
    TopoDS_Shape slitCutter;
    try {
        slitCutter = pr::box(slitAx, slitKerf, slitLen, (zMax - zMin) + 0.1);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("lance_form: slit cutter failed: ") + ex.what());
    }

    TopoDS_Shape afterSlit;
    try {
        afterSlit = pr::cut(wp.shape(), slitCutter);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("lance_form: slit cut failed: ") + ex.what());
    }

    // 2. Fuse a raised rectangular tab on one side of the slit.  Tab base
    //    sits at z = zMax (sheet top), extruding by form_height_mm.  Tab
    //    centred along the slit, offset perp by form_width_mm / 2.
    const gp_Pnt tabOrigin(
        pA.X() + perp.X() * (slitKerf * 0.5),
        pA.Y() + perp.Y() * (slitKerf * 0.5),
        zMax);
    const gp_Ax2 tabAx(tabOrigin, gp::DZ(), gp_Dir(perp));
    TopoDS_Shape tab;
    try {
        tab = pr::box(tabAx, in.form_width_mm, slitLen, in.form_height_mm);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("lance_form: tab box failed: ") + ex.what());
    }

    TopoDS_Shape result;
    try {
        result = pr::fuse(afterSlit, tab);
    } catch (const Standard_Failure& ex) {
        spdlog::warn("lance_form: tab fuse failed: {}; using slit-only", ex.what());
        result = afterSlit;
    }

    // Signature.
    json params = {
        { "slit_start_xy",  { in.slit_start_xy[0], in.slit_start_xy[1] } },
        { "slit_end_xy",    { in.slit_end_xy[0],   in.slit_end_xy[1]   } },
        { "form_height_mm", in.form_height_mm },
        { "form_width_mm",  in.form_width_mm  },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "sheet_feature_type",    "lance_and_form" },
        { "slit_length_mm",        slitLen },
        { "slit_kerf_mm",          slitKerf },
        { "form_height_mm",        in.form_height_mm },
        { "form_width_mm",         in.form_width_mm  },
        { "tab_volume_mm3",        in.form_width_mm * slitLen * in.form_height_mm },
        { "sheet_thickness_mm",    t },
        { "slit_axis_dir",         { dirV.X(), dirV.Y(), dirV.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lance_form_die";
    tooling.tool_dia_mm       = in.form_width_mm;
    tooling.tool_length_mm    = slitLen;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = slitKerf * slitLen * t;
    tooling.est_cycle_time_s  = 1.5;       // single press stroke
    tooling.extra["machining_constraint"] =
        "Lance-and-form — combined shear + form in one die hit; "
        "slit > 5× thickness for clean shear; tab height < 4× thickness "
        "to avoid tear at slit root";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::lance_form applied: slit={} tab={}×{} faces {}→{}",
                  slitLen, in.form_width_mm, in.form_height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A lance-and-form leaves a raised rectangular planar patch (+Z normal)
// above the sheet plane, adjacent to a through-cut slit.  We scan for any
// localized +Z planar face at Z > sheet_top + thickness × 0.5.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 6.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    (void)xMin; (void)yMin; (void)zMin; (void)xMax; (void)yMax; (void)zMax;

    // Find LARGEST +Z planar face at the "main" sheet top Z.
    int    sheetTopId  = -1;
    double sheetTopArea = -1.0;
    double sheetTopZ    = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.85) continue;
        const double a = wp.faceArea(i);
        if (a > sheetTopArea) {
            sheetTopArea = a;
            sheetTopId   = i;
            sheetTopZ    = wp.faceCenter(i).Z();
        }
    }
    if (sheetTopId < 0) return out;

    // Find any SMALLER +Z planar face raised above sheet top by ≥ 0.5·t.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (i == sheetTopId) continue;
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.85) continue;
        const double area = wp.faceArea(i);
        if (area >= sheetTopArea * 0.5) continue;     // tab must be smaller
        const double zC  = wp.faceCenter(i).Z();
        const double sep = zC - sheetTopZ;
        if (sep < 0.5 * t) continue;
        if (sep > 5.0 * t) continue;
        const gp_Pnt c = wp.faceCenter(i);

        json recovered = {
            { "slit_start_xy",  { c.X() - 1.0, c.Y() } },
            { "slit_end_xy",    { c.X() + 1.0, c.Y() } },
            { "form_height_mm", sep },
            { "form_width_mm",  std::sqrt(area) },
        };
        json matched = {
            { "tab_face_id",     i },
            { "tab_area_mm2",    area },
            { "tab_height_mm",   sep },
            { "sheet_top_face_id", sheetTopId },
            { "sheet_thickness_mm", t },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::lance_form
