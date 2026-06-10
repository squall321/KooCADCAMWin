// @lat: [[engine/skills#lance_n_form]]

#include "lance_n_form.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::lance_n_form {

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

    if (in.tab_length_mm <= 0.0 || in.tab_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lance_n_form requires tab_length_mm > 0 and tab_width_mm > 0");
    }
    if (in.form_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lance_n_form requires form_height_mm > 0");
    }
    if (in.lance_kerf_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lance_n_form requires lance_kerf_mm > 0");
    }

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-LNF-SHEET", "error",
              "lance_n_form: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t <= 0.0) {
        r.add("DFM-LNF-SHEET", "error",
              "lance_n_form: degenerate stock thickness");
    }

    const double minTab = std::min(in.tab_length_mm, in.tab_width_mm);
    if (minTab > 0.0 && t > 0.0 && minTab < 2.0 * t) {
        r.add("DFM-LNF-MIN-TAB", "error",
              "lance_n_form tab min dim " + std::to_string(minTab) +
              " mm < 2 × sheet_thickness (" + std::to_string(2.0 * t) +
              " mm) — tab too small to lance + form");
    }

    if (in.form_height_mm > 0.0 && t > 0.0 && in.form_height_mm > 4.0 * t) {
        r.add("DFM-LNF-MAX-FORM", "error",
              "lance_n_form form_height " + std::to_string(in.form_height_mm) +
              " mm > 4 × sheet_thickness (" + std::to_string(4.0 * t) +
              " mm) — tear / crack risk at hinge");
    }

    if (in.lance_kerf_mm > 0.0 && t > 0.0 && in.lance_kerf_mm < 0.5 * t) {
        r.add("DFM-LNF-KERF", "error",
              "lance_n_form kerf " + std::to_string(in.lance_kerf_mm) +
              " mm < 0.5 × sheet_thickness (" + std::to_string(0.5 * t) +
              " mm) — kerf too narrow for punch tooling");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Geometry: cut three slots around the tab footprint (skipping the hinge
// side), then fuse a small upright box (the bent tab) at the hinge edge.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lance_n_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t       = sheetThicknessOf(wp);
    const double zSpan   = (zMax - zMin) + 0.2;
    const double zStart  = zMin - 0.1;

    const double halfL = in.tab_length_mm / 2.0;
    const double halfW = in.tab_width_mm  / 2.0;
    const double k     = in.lance_kerf_mm;
    const double cx    = in.tab_center_x_mm;
    const double cy    = in.tab_center_y_mm;

    // Three lance slot cutters.  Skip the slot corresponding to the hinge.
    std::vector<TopoDS_Shape> slots;
    auto addSlot = [&](const gp_Pnt& origin, double dx, double dy) {
        slots.push_back(pr::box(gp_Ax2(origin, gp::DZ()), dx, dy, zSpan));
    };
    // Slot on +X side: thin in X, runs full Y extent of tab.
    if (in.hinge_side != HingeSide::PlusX) {
        addSlot(gp_Pnt(cx + halfL - k / 2.0,
                       cy - halfW,
                       zStart),
                k, in.tab_width_mm);
    }
    // Slot on -X side.
    if (in.hinge_side != HingeSide::MinusX) {
        addSlot(gp_Pnt(cx - halfL - k / 2.0,
                       cy - halfW,
                       zStart),
                k, in.tab_width_mm);
    }
    // Slot on +Y side.
    if (in.hinge_side != HingeSide::PlusY) {
        addSlot(gp_Pnt(cx - halfL,
                       cy + halfW - k / 2.0,
                       zStart),
                in.tab_length_mm, k);
    }
    // Slot on -Y side.
    if (in.hinge_side != HingeSide::MinusY) {
        addSlot(gp_Pnt(cx - halfL,
                       cy - halfW - k / 2.0,
                       zStart),
                in.tab_length_mm, k);
    }

    TopoDS_Shape sheetCut;
    try {
        sheetCut = pr::cutMany(wp.shape(), slots);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("lance_n_form: lance cut failed: ") + ex.what());
    }

    // Additive: upright box for the bent tab on top of the sheet.  Placed
    // near the hinge edge so it sticks up from the un-cut side.
    double tabX = cx - halfL;
    double tabY = cy - halfW;
    double tabDx = in.tab_length_mm;
    double tabDy = in.tab_width_mm;
    // Reduce one dimension to approximate the bent tab "thickness" (it
    // becomes a thin wall after forming).
    const double wallT = t;
    switch (in.hinge_side) {
        case HingeSide::PlusY:
            tabY  = cy + halfW - wallT;
            tabDy = wallT;
            break;
        case HingeSide::MinusY:
            tabY  = cy - halfW;
            tabDy = wallT;
            break;
        case HingeSide::PlusX:
            tabX  = cx + halfL - wallT;
            tabDx = wallT;
            break;
        case HingeSide::MinusX:
            tabX  = cx - halfL;
            tabDx = wallT;
            break;
    }

    TopoDS_Shape result = sheetCut;
    try {
        TopoDS_Shape bentTab = pr::box(
            gp_Ax2(gp_Pnt(tabX, tabY, zMax), gp::DZ()),
            tabDx, tabDy, in.form_height_mm);
        result = pr::fuse(sheetCut, bentTab);
    } catch (const Standard_Failure& ex) {
        spdlog::warn("lance_n_form: bent-tab fuse failed ({}); keeping cut-only result",
                     ex.what());
    }

    // Signature
    json params = {
        { "tab_center_x_mm", in.tab_center_x_mm },
        { "tab_center_y_mm", in.tab_center_y_mm },
        { "tab_length_mm",   in.tab_length_mm },
        { "tab_width_mm",    in.tab_width_mm },
        { "form_height_mm",  in.form_height_mm },
        { "lance_kerf_mm",   in.lance_kerf_mm },
        { "hinge_side",      hingeSideToString(in.hinge_side) },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "tab_length_mm",        in.tab_length_mm },
        { "tab_width_mm",         in.tab_width_mm },
        { "form_height_mm",       in.form_height_mm },
        { "lance_kerf_mm",        in.lance_kerf_mm },
        { "hinge_side",           hingeSideToString(in.hinge_side) },
        { "sheet_thickness_mm",   t },
        { "has_bent_tab",         true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lance_form_die";
    tooling.tool_dia_mm       = in.lance_kerf_mm;
    tooling.tool_length_mm    = in.form_height_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 3.0 * std::max(in.tab_length_mm, in.tab_width_mm)
                                * in.lance_kerf_mm * t;
    tooling.est_cycle_time_s  = 1.0;
    tooling.extra["machining_constraint"] =
        "Lance-and-form — single-die compound: punch lances 3 sides while "
        "form-shoulder bends the freed tab; hinge edge stays uncut";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::lance_n_form applied: tab={}x{} h={} hinge={}",
                  in.tab_length_mm, in.tab_width_mm, in.form_height_mm,
                  hingeSideToString(in.hinge_side));

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Lance-and-form features are complex compound features; reliable
// geometric recovery would require detecting both the slit kerfs and the
// upright tab.  For slice-1 we rely on metadata replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            json{{ "source", "metadata" }}});
    }
    return out;
}

}  // namespace koocadcam::skill::lance_n_form
