// @lat: [[engine/skills#slot_weld_slot]]

#include "slot_weld_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

namespace koocadcam::skill::slot_weld_slot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.slot_w < 3.0) {
        r.add("DFM-WELD-SLOT-WIDTH", "error",
              "slot_weld_slot: slot_w " + std::to_string(in.slot_w) +
              " mm < 3 mm minimum (AWS A2.4)");
    }
    if (in.slot_l < 2.0 * in.slot_w) {
        r.add("DFM-WELD-SLOT-LW", "error",
              "slot_weld_slot: slot_l " + std::to_string(in.slot_l) +
              " mm < 2 × slot_w " + std::to_string(2.0 * in.slot_w) +
              " — use plug_weld_hole instead");
    }
    if (in.slot_w <= 0.0 || in.slot_l <= 0.0) {
        r.add("DFM-INPUT", "error",
              "slot_weld_slot: slot_l and slot_w must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "slot_weld_slot DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId)
        throw SkillError("slot_weld_slot: entry_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const gp_Dir adir = in.axis_dir;
    constexpr double kOverhang = 0.05;
    double entryZ = zMax;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        entryZ = (adir.Z() < 0) ? zMax : zMin;
    }

    // Local frame for slot orientation: xLoc = slot_dir, yLoc = 90° CCW.
    const gp_Vec slotVec(in.slot_dir.X(), in.slot_dir.Y(), 0.0);
    if (slotVec.Magnitude() < 1e-6)
        throw SkillError("slot_weld_slot: slot_dir is vertical");
    const gp_Dir xLoc(slotVec.X(), slotVec.Y(), 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    const double cylR    = in.slot_w / 2.0;
    const double boxLen  = in.slot_l - in.slot_w;   // length of straight section
    if (boxLen <= 0.0)
        throw SkillError("slot_weld_slot: slot_l <= slot_w (degenerate)");

    const double cutH = bboxDiag + 2.0 * kOverhang;

    // ── Sub-feature 1: center rectangular box ──────────────────────────
    //
    // gp_Ax2(P, V=adir, Vx=xLoc): DX along xLoc (boxLen),
    //                              DY along yLoc (slot_w),
    //                              DZ along adir (cutH).
    const gp_Pnt boxOrigin(
        in.position_x_mm - xLoc.X() * boxLen / 2.0 - yLoc.X() * cylR,
        in.position_y_mm - xLoc.Y() * boxLen / 2.0 - yLoc.Y() * cylR,
        entryZ - adir.Z() * kOverhang);
    const gp_Ax2 boxAx(boxOrigin, adir, xLoc);
    const TopoDS_Shape boxCut = pr::box(boxAx, boxLen, in.slot_w, cutH);

    // ── Sub-feature 2: end cylinder A (at +xLoc end) ──────────────────
    const gp_Pnt cylAStart(
        in.position_x_mm + xLoc.X() * boxLen / 2.0,
        in.position_y_mm + xLoc.Y() * boxLen / 2.0,
        entryZ - adir.Z() * kOverhang);
    const gp_Ax2 cylAAx(cylAStart, adir);
    const TopoDS_Shape cylA = pr::cylinder(cylAAx, cylR, cutH);

    // ── Sub-feature 3: end cylinder B (at -xLoc end) ──────────────────
    const gp_Pnt cylBStart(
        in.position_x_mm - xLoc.X() * boxLen / 2.0,
        in.position_y_mm - xLoc.Y() * boxLen / 2.0,
        entryZ - adir.Z() * kOverhang);
    const gp_Ax2 cylBAx(cylBStart, adir);
    const TopoDS_Shape cylB = pr::cylinder(cylBAx, cylR, cutH);

    // Fuse three overlapping cutters → single cut.
    const TopoDS_Shape fused1   = pr::fuse(boxCut, cylA);
    const TopoDS_Shape fused2   = pr::fuse(fused1, cylB);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused2);

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "slot_l",         in.slot_l },
        { "slot_w",         in.slot_w },
        { "slot_dir",       { in.slot_dir.X(), in.slot_dir.Y(), in.slot_dir.Z() } },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "subfeature_count", 3 },
        { "slot_l",           in.slot_l },
        { "slot_w",           in.slot_w },
        { "corner_r_mm",      cylR },                 // derived
        { "straight_len_mm",  boxLen },               // derived
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "slot_dir",         { in.slot_dir.X(), in.slot_dir.Y(), in.slot_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill";
    tooling.tool_dia_mm       = in.slot_w;
    tooling.tool_length_mm    = 15.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double slotArea =
        boxLen * in.slot_w + M_PI * cylR * cylR;     // stadium area
    tooling.stock_removed_mm3 = slotArea * (zMax - zMin);
    tooling.est_cycle_time_s  = std::max(2.0, in.slot_l / 30.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill"    }, { "pass", "end_A" } },
            { { "tool_type", "drill"    }, { "pass", "end_B" } },
            { { "tool_type", "end_mill" }, { "pass", "center_box" },
              { "tool_dia_mm", in.slot_w } },
        } },
        { "aws_classification", "slot_weld" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::slot_weld_slot applied: L={} W={} center=({},{})",
                  in.slot_l, in.slot_w, in.position_x_mm, in.position_y_mm);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& s : wp.features()) {
        if (s.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = s.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::slot_weld_slot
