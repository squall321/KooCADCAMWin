// @lat: [[engine/skills#over_center_toggle_pocket]]

#include "over_center_toggle_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::over_center_toggle_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec table (DIN 6840 toggle-clamp pocket dims) ───────────────────────

namespace {

constexpr std::array<ToggleSpec, 3> kToggleTable {{
    { "TC-A20", 3.0, 10.0, 2.0, 2.0, 1.0 },
    { "TC-A40", 5.0, 18.0, 3.0, 3.0, 1.5 },
    { "TC-A80", 8.0, 30.0, 5.0, 5.0, 2.5 },
}};

}  // namespace

const ToggleSpec* lookupToggleSpec(const std::string& key)
{
    for (const auto& s : kToggleTable) {
        if (key == s.key) return &s;
    }
    return nullptr;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const ToggleSpec* spec = lookupToggleSpec(in.toggle_class);
    if (!spec) {
        r.add("DFM-SPEC", "error",
              "over_center_toggle_pocket: unknown toggle_class '" +
              in.toggle_class + "' (expected TC-A20|TC-A40|TC-A80)");
        return r;
    }

    if (in.pocket_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "over_center_toggle_pocket: pocket_depth_mm must be > 0");
    }
    if (spec->slot_width_mm < 0.8 || spec->anchor_dia_mm < 0.8
        || spec->stop_width_mm < 0.8 || spec->pivot_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "over_center_toggle_pocket: a sub-dim < 0.8 mm in spec '" +
              in.toggle_class + "'");
    }
    // DIN 6840: pivot bore depth must be ≥ 1.5 × pivot_dia for proper engagement.
    if (in.pocket_depth_mm < spec->pivot_dia_mm * 1.5) {
        r.add("DFM-PIVOT", "error",
              "over_center_toggle_pocket: pocket_depth " +
              std::to_string(in.pocket_depth_mm) +
              " mm < 1.5 × pivot_dia " +
              std::to_string(spec->pivot_dia_mm * 1.5) + " mm");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "over_center_toggle_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ToggleSpec* spec = lookupToggleSpec(in.toggle_class);
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("over_center_toggle_pocket: entry_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Local frame: slot rotated by slot_angle_deg about pivot.
    const double a = in.slot_angle_deg * M_PI / 180.0;
    const gp_Dir xLoc(std::cos(a), std::sin(a), 0.0);
    const gp_Dir yLoc(-std::sin(a), std::cos(a), 0.0);

    // SUBFEATURE 1: PIVOT BORE
    const gp_Pnt pivotTop(in.pivot_x_mm, in.pivot_y_mm, zMax + kOverhang);
    const gp_Ax2 pivotAx(pivotTop, gp_Dir(0.0, 0.0, -1.0), gp::DX());
    const TopoDS_Shape pivotBore = pr::cylinder(
        pivotAx, spec->pivot_dia_mm / 2.0, in.pocket_depth_mm + 2.0 * kOverhang);

    // SUBFEATURE 2: CAM SLOT — offset from pivot by slot_length/2 along +xLoc,
    // rectangular slot, oriented along xLoc.
    const double slotMidDist = spec->slot_length_mm * 0.75;
    const gp_Pnt slotMid(
        in.pivot_x_mm + slotMidDist * xLoc.X(),
        in.pivot_y_mm + slotMidDist * xLoc.Y(),
        zMax);
    const gp_Pnt slotOrigin(
        slotMid.X() - spec->slot_length_mm / 2.0 * xLoc.X()
                    - spec->slot_width_mm  / 2.0 * yLoc.X(),
        slotMid.Y() - spec->slot_length_mm / 2.0 * xLoc.Y()
                    - spec->slot_width_mm  / 2.0 * yLoc.Y(),
        zMax - in.pocket_depth_mm);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ(), xLoc);
    const TopoDS_Shape slotBox = pr::box(
        slotAx, spec->slot_length_mm, spec->slot_width_mm,
        in.pocket_depth_mm + kOverhang);

    // SUBFEATURE 3: OVER-CENTER STOP — small notch at the slot midpoint that
    // gives the "snap".  Modelled as a small box centered on slotMid.
    const double stopL = spec->slot_width_mm * 1.5;
    const gp_Pnt stopOrigin(
        slotMid.X() - stopL                 / 2.0 * xLoc.X()
                    - spec->stop_width_mm * 1.5 * yLoc.X(),
        slotMid.Y() - stopL                 / 2.0 * xLoc.Y()
                    - spec->stop_width_mm * 1.5 * yLoc.Y(),
        zMax - in.pocket_depth_mm * 0.6);
    const gp_Ax2 stopAx(stopOrigin, gp::DZ(), xLoc);
    const TopoDS_Shape stopBox = pr::box(
        stopAx, stopL, spec->stop_width_mm * 3.0, in.pocket_depth_mm * 0.6 + kOverhang);

    // SUBFEATURE 4: SPRING ANCHOR HOLE — small bore on the opposite side of
    // pivot from the slot.
    const gp_Pnt anchorTop(
        in.pivot_x_mm - slotMidDist * xLoc.X(),
        in.pivot_y_mm - slotMidDist * xLoc.Y(),
        zMax + kOverhang);
    const gp_Ax2 anchorAx(anchorTop, gp_Dir(0.0, 0.0, -1.0), gp::DX());
    const TopoDS_Shape anchorBore = pr::cylinder(
        anchorAx, spec->anchor_dia_mm / 2.0, in.pocket_depth_mm + 2.0 * kOverhang);

    // Boolean composition.
    const TopoDS_Shape c1     = pr::fuse(pivotBore, slotBox);
    const TopoDS_Shape c2     = pr::fuse(c1, stopBox);
    const TopoDS_Shape cutter = pr::fuse(c2, anchorBore);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",    *entryId },
        { "pivot_x_mm",       in.pivot_x_mm },
        { "pivot_y_mm",       in.pivot_y_mm },
        { "axis_dir",         { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "pocket_depth_mm",  in.pocket_depth_mm },
        { "slot_angle_deg",   in.slot_angle_deg },
        { "toggle_class",     in.toggle_class },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },
        { "toggle_class",        in.toggle_class },
        { "pivot_dia_mm",        spec->pivot_dia_mm },
        { "slot_length_mm",      spec->slot_length_mm },
        { "slot_width_mm",       spec->slot_width_mm },
        { "anchor_dia_mm",       spec->anchor_dia_mm },
        { "stop_width_mm",       spec->stop_width_mm },
        { "pocket_depth_mm",     in.pocket_depth_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type = "drill;end_mill;ball_mill";
    tooling.tool_dia_mm = std::max(spec->pivot_dia_mm, spec->slot_width_mm);
    tooling.tool_length_mm = in.pocket_depth_mm + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 3;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.03;
    const double rPivot = spec->pivot_dia_mm / 2.0;
    const double rAnchor = spec->anchor_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        M_PI * rPivot * rPivot * in.pocket_depth_mm +
        spec->slot_length_mm * spec->slot_width_mm * in.pocket_depth_mm +
        stopL * spec->stop_width_mm * 3.0 * in.pocket_depth_mm * 0.6 +
        M_PI * rAnchor * rAnchor * in.pocket_depth_mm;
    tooling.est_cycle_time_s = std::max(10.0, in.pocket_depth_mm * 1.5);
    tooling.extra = {
        { "spec_table_ref", "DIN 6840 — toggle clamp pocket sizes" },
        { "compound_chain", {
            { { "step", "pivot_bore" },     { "tool", "drill" } },
            { { "step", "cam_slot" },       { "tool", "end_mill" } },
            { { "step", "over_center_stop" }, { "tool", "end_mill" } },
            { { "step", "spring_anchor" },  { "tool", "drill" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::over_center_toggle_pocket applied: class={} depth={}",
                  in.toggle_class, in.pocket_depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id = kSkillId;
        r.confidence = 0.99;
        r.recovered_params = f.params;
        r.matched_geometry = {
            { "source", "metadata_replay" },
            { "subfeature_count", f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric heuristic: ≥ 2 cylindrical faces (pivot + anchor), with
    // matching axes parallel to Z, at distinct XY positions.
    std::vector<std::tuple<int, double, gp_Pnt>> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder cyl = s.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-2) continue;
        cyls.emplace_back(i, 2.0 * cyl.Radius(), cyl.Location());
    }
    if (cyls.size() < 2) return out;

    // Find pivot (larger) and anchor (smaller).
    std::sort(cyls.begin(), cyls.end(),
              [](const auto& a, const auto& b) {
                  return std::get<1>(a) > std::get<1>(b);
              });
    const double pivotDia  = std::get<1>(cyls[0]);
    const double anchorDia = std::get<1>(cyls[1]);
    const gp_Pnt pivotLoc  = std::get<2>(cyls[0]);

    const ToggleSpec* best = nullptr;
    double bestErr = 1e9;
    for (const auto& s : kToggleTable) {
        const double err = std::abs(s.pivot_dia_mm - pivotDia) +
                           std::abs(s.anchor_dia_mm - anchorDia);
        if (err < bestErr) { bestErr = err; best = &s; }
    }
    if (!best || bestErr > 2.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    json recovered = {
        { "pivot_x_mm",      pivotLoc.X() },
        { "pivot_y_mm",      pivotLoc.Y() },
        { "axis_dir",        { 0.0, 0.0, -1.0 } },
        { "pocket_depth_mm", std::max(1.0, (zMax - zMin) * 0.6) },
        { "slot_angle_deg",  0.0 },
        { "toggle_class",    std::string(best->key) },
    };
    json matched = {
        { "pivot_face_id",  std::get<0>(cyls[0]) },
        { "anchor_face_id", std::get<0>(cyls[1]) },
        { "cyl_count",      static_cast<int>(cyls.size()) },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.62, matched });
    return out;
}

}  // namespace koocadcam::skill::over_center_toggle_pocket
