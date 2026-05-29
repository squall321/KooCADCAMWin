// @lat: [[engine/skills#eccentric_bushing_seat]]

#include "eccentric_bushing_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::eccentric_bushing_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.bore_od_mm < 4.0) {
        r.add("DFM-EB-DIA", "error",
              "eccentric_bushing_seat: bore_od_mm " +
              std::to_string(in.bore_od_mm) + " < 4 mm");
    }
    if (in.bore_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "eccentric_bushing_seat: bore_depth_mm must be > 0");
    }
    if (in.slot_arc_deg < 10.0 || in.slot_arc_deg > 120.0) {
        r.add("DFM-EB-ARC", "error",
              "eccentric_bushing_seat: slot_arc_deg " +
              std::to_string(in.slot_arc_deg) +
              "° outside [10°, 120°] (provides no useful adjustment "
              "range OR removes too much bore wall)");
    }
    if (in.slot_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "eccentric_bushing_seat: slot_width_mm " +
              std::to_string(in.slot_width_mm) + " < 0.8 mm (min end-mill)");
    }
    if (in.slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "eccentric_bushing_seat: slot_depth_mm must be > 0");
    }
    if (in.chamfer_size_mm < 0.1) {
        r.add("DFM-EB-CHAMFER", "warning",
              "eccentric_bushing_seat: chamfer too small (< 0.1 mm), "
              "knife-edge risk on bore rim");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "eccentric_bushing_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(
        "eccentric_bushing_seat: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt entryPnt;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0)
            entryPnt = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        else
            entryPnt = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
    } else {
        entryPnt = gp_Pnt(in.position_x_mm, in.position_y_mm, (zMin + zMax) / 2.0);
    }

    const gp_Ax2 axMain(entryPnt, adir);
    const double boreR = in.bore_od_mm / 2.0;

    // ── Sub-feature 1: bushing bore (cylinder) ─────────────────────────
    const TopoDS_Shape boreCutter =
        pr::cylinder(axMain, boreR, in.bore_depth_mm + kOverhang);

    // ── Sub-features 2 + 3: two arc slots at 0° / 180° ─────────────────
    // We approximate each arc slot with an oriented box tangent to the bore
    // at the slot mid-angle.  The chord length of the arc at radius `boreR`
    // is 2 × boreR × sin(arc/2).  Box dims:
    //   - length  = chord
    //   - width   = slot_width_mm  (radial thickness)
    //   - height  = slot_depth_mm + overhang
    //
    // gp_Ax2(P, V=axisLocalZ, Vx=localX).  V = adir, localX along the
    // bore-tangent direction at the slot mid-angle.
    const double arcRad = in.slot_arc_deg * M_PI / 180.0;
    const double chord  = 2.0 * boreR * std::sin(arcRad / 2.0);

    auto makeArcSlotCutter = [&](double centerAngleRad) -> TopoDS_Shape {
        // Mid-angle point on the bore circle (XY plane, at entry Z).
        const double midX = in.position_x_mm + boreR * std::cos(centerAngleRad);
        const double midY = in.position_y_mm + boreR * std::sin(centerAngleRad);
        // Tangent direction = perpendicular to radial.
        const gp_Dir localX(-std::sin(centerAngleRad),
                             std::cos(centerAngleRad), 0.0);
        // localY (= localZ × localX, but in OCCT gp_Ax2(P, V, Vx) V=local-Z;
        // localY = V × Vx).  For us local-Z = adir, local-X = tangent.
        // Origin of the box at (midPoint shifted by -chord/2 along
        // localX and -slot_width/2 along radial) then up by slot_depth
        // against adir.
        // Radial outward direction in XY (from bore centre to mid point):
        const gp_Dir radial(std::cos(centerAngleRad),
                             std::sin(centerAngleRad), 0.0);
        // Origin: mid - chord/2 × localX - slot_width/2 × radial,
        //         z = entry plane (against adir we extrude DZ = depth).
        gp_Pnt boxOrigin(
            midX - chord / 2.0 * localX.X() - in.slot_width_mm / 2.0 * radial.X(),
            midY - chord / 2.0 * localX.Y() - in.slot_width_mm / 2.0 * radial.Y(),
            entryPnt.Z());
        // Extrude along -adir's reverse → box dz is slot_depth in +Z if
        // adir is -Z (top-face entry).  We use adir reversed for box origin
        // so DZ goes against adir … but pr::box uses gp_Ax2(P, V, Vx) with
        // V = local Z direction and DZ extrudes along V.  So set V = -adir
        // (into the material) and DX = chord, DY = slot_width, DZ =
        // slot_depth.
        const gp_Dir boxV(-adir.X(), -adir.Y(), -adir.Z());
        // We want DZ to go into material; gp_Ax2(P, boxV, localX) so DZ
        // along boxV.  Adjust origin so the entry plane is at z = entryPnt.Z()
        // and extrusion goes (slot_depth + overhang) into the material.
        const gp_Ax2 boxAx(boxOrigin, boxV, localX);
        return pr::box(boxAx, chord, in.slot_width_mm,
                        in.slot_depth_mm + kOverhang);
    };

    const TopoDS_Shape slotA = makeArcSlotCutter(0.0);
    const TopoDS_Shape slotB = makeArcSlotCutter(M_PI);

    // ── Sub-feature 4: retention chamfer (cone frustum, rim) ───────────
    // Cone goes from boreR (small, at entry) to boreR + chamfer (large,
    // OUTSIDE the bore, above entry plane).  Cutter removes material from
    // the rim outward.
    gp_Pnt chamferStart = entryPnt;
    chamferStart.SetX(chamferStart.X() - adir.X() * in.chamfer_size_mm);
    chamferStart.SetY(chamferStart.Y() - adir.Y() * in.chamfer_size_mm);
    chamferStart.SetZ(chamferStart.Z() - adir.Z() * in.chamfer_size_mm);
    const gp_Ax2 chamferAx(chamferStart, adir);
    const TopoDS_Shape chamferCutter =
        pr::coneFrustum(chamferAx, boreR + in.chamfer_size_mm,
                                    boreR,
                                    in.chamfer_size_mm);

    // Fuse all cutters then single cut.
    const TopoDS_Shape fused1 = pr::fuse(boreCutter, chamferCutter);
    const TopoDS_Shape fused2 = pr::fuse(fused1, slotA);
    const TopoDS_Shape fused3 = pr::fuse(fused2, slotB);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused3);

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "bore_od_mm",      in.bore_od_mm },
        { "bore_depth_mm",   in.bore_depth_mm },
        { "slot_arc_deg",    in.slot_arc_deg },
        { "slot_width_mm",   in.slot_width_mm },
        { "slot_depth_mm",   in.slot_depth_mm },
        { "chamfer_size_mm", in.chamfer_size_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  4 },
        { "bore_od_mm",        in.bore_od_mm },
        { "bore_depth_mm",     in.bore_depth_mm },
        { "slot_arc_deg",      in.slot_arc_deg },
        { "slot_chord_mm",     chord },
        { "slot_width_mm",     in.slot_width_mm },
        { "slot_depth_mm",     in.slot_depth_mm },
        { "chamfer_size_mm",   in.chamfer_size_mm },
        { "slot_angles_deg",   { 0.0, 180.0 } },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;end_mill;chamfer";
    tooling.tool_dia_mm       = in.bore_od_mm;
    tooling.tool_length_mm    = in.bore_depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 240.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 =
        M_PI * boreR * boreR * in.bore_depth_mm +
        2.0 * chord * in.slot_width_mm * in.slot_depth_mm +
        2.0 * M_PI * boreR * in.chamfer_size_mm * in.chamfer_size_mm * 0.5;
    tooling.est_cycle_time_s = std::max(5.0,
        in.bore_depth_mm * 0.4 + 4.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" },
              { "tool_dia_mm", in.bore_od_mm },
              { "depth_mm",    in.bore_depth_mm },
              { "purpose",     "bushing seat bore" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.slot_width_mm },
              { "depth_mm",    in.slot_depth_mm },
              { "arc_deg",     in.slot_arc_deg },
              { "purpose",     "adjustment arc slot A (0°)" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.slot_width_mm },
              { "depth_mm",    in.slot_depth_mm },
              { "arc_deg",     in.slot_arc_deg },
              { "purpose",     "adjustment arc slot B (180°)" } },
            { { "tool_type", "chamfer_tool" },
              { "tool_dia_mm", in.bore_od_mm + 2.0 * in.chamfer_size_mm },
              { "depth_mm",    in.chamfer_size_mm },
              { "angle_deg",   45.0 },
              { "purpose",     "retention chamfer on bore rim" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::eccentric_bushing_seat applied: OD={} depth={} arc={}",
                  in.bore_od_mm, in.bore_depth_mm, in.slot_arc_deg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition — metadata replay ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "via", "metadata_replay" },
              { "subfeature_count", 4 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::eccentric_bushing_seat
