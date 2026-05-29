// @lat: [[engine/skills#pcb_card_edge_socket]]

#include "pcb_card_edge_socket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::pcb_card_edge_socket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    static const double allowedPitches[] = { 1.27, 2.0, 2.54 };
    bool pitchOk = false;
    for (double p : allowedPitches) if (std::abs(p - in.pitch_mm) < 0.01) { pitchOk = true; break; }
    if (!pitchOk) {
        r.add("DFM-JEDEC-MO189", "error",
              "pitch " + std::to_string(in.pitch_mm) +
              " mm not in {1.27, 2.0, 2.54}");
    }
    if (in.finger_count <= 0) {
        r.add("DFM-INPUT", "error", "finger_count must be > 0");
    }
    if (in.finger_dia_mm >= in.pitch_mm * 0.8) {
        r.add("DFM-CARDEDGE-CLR", "error",
              "finger_dia " + std::to_string(in.finger_dia_mm) +
              " mm too large for pitch (< 0.8×pitch required)");
    }
    if (in.slot_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "slot_width " + std::to_string(in.slot_width_mm) +
              " mm < 0.8 mm");
    }
    if (in.slot_depth_mm <= 0.0 || in.finger_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "slot_depth and finger_depth must be > 0");
    }
    if (in.finger_depth_mm < in.slot_depth_mm * 0.5) {
        r.add("DFM-CARDEDGE-FINGER", "warning",
              "finger_depth shallow relative to slot — contact bend may not seat");
    }
    const double slotLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                      in.end_y_mm - in.start_y_mm);
    if (slotLen < in.finger_count * in.pitch_mm) {
        r.add("DFM-CARDEDGE-PITCH", "error",
              "slot length " + std::to_string(slotLen) +
              " mm too short for " + std::to_string(in.finger_count) +
              " fingers at pitch " + std::to_string(in.pitch_mm));
    }
    if (in.key_tab_offset_mm <= 0.0 || in.key_tab_offset_mm >= slotLen) {
        r.add("DFM-CARDEDGE-KEY", "error",
              "key_tab_offset must be inside slot length");
    }
    if (in.key_tab_width_mm <= 0.0 || in.key_tab_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "key_tab dimensions must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pcb_card_edge_socket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("pcb_card_edge_socket: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("pcb_card_edge_socket: only axis_dir = -Z supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zEntry = zMax;
    const double kOverhang = 0.05;

    // Local frame.
    const double dx2 = in.end_x_mm - in.start_x_mm;
    const double dy2 = in.end_y_mm - in.start_y_mm;
    const double slotLen = std::hypot(dx2, dy2);
    const gp_Dir xLoc(dx2 / slotLen, dy2 / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    // 1) Main slot — rectangular pocket extending along xLoc by slotLen,
    //    width = slot_width, depth = slot_depth.
    const gp_Pnt slotOrigin(
        in.start_x_mm - in.slot_width_mm / 2.0 * yLoc.X(),
        in.start_y_mm - in.slot_width_mm / 2.0 * yLoc.Y(),
        zEntry - in.slot_depth_mm);
    const gp_Ax2 slotAx(slotOrigin, gp_Dir(0, 0, 1), xLoc);
    const TopoDS_Shape slot = pr::box(
        slotAx, slotLen, in.slot_width_mm,
        in.slot_depth_mm + kOverhang);

    // 2) N contact-finger reliefs — small cylindrical pockets along the +yLoc
    //    wall of the slot, at pitch intervals.  Each relief sits OUTSIDE
    //    the slot (extending into the wall) and is bored downward.
    const double pitch = in.pitch_mm;
    const double rowOffset = in.slot_width_mm / 2.0 + in.finger_dia_mm / 2.0;
    const double rowStart = (slotLen - (in.finger_count - 1) * pitch) / 2.0;
    std::vector<TopoDS_Shape> fingers;
    fingers.reserve(in.finger_count);
    for (int i = 0; i < in.finger_count; ++i) {
        const double along = rowStart + i * pitch;
        const gp_Pnt fc(
            in.start_x_mm + along * xLoc.X() + rowOffset * yLoc.X(),
            in.start_y_mm + along * xLoc.Y() + rowOffset * yLoc.Y(),
            zEntry + kOverhang);
        const gp_Ax2 fAx(fc, gp_Dir(0, 0, -1));
        fingers.push_back(pr::cylinder(
            fAx, in.finger_dia_mm / 2.0,
            in.finger_depth_mm + kOverhang));
    }

    // 3) Key tab notch — extra slot widening at key_tab_offset along the slot.
    //    Modeled as a rectangular box that extends the slot width on BOTH
    //    sides at the offset position.
    const double notchAlong = in.key_tab_offset_mm;
    const double totalNotchWidth = in.slot_width_mm + 2.0 * in.key_tab_depth_mm;
    const gp_Pnt notchOrigin(
        in.start_x_mm + (notchAlong - in.key_tab_width_mm / 2.0) * xLoc.X()
                       - totalNotchWidth / 2.0 * yLoc.X(),
        in.start_y_mm + (notchAlong - in.key_tab_width_mm / 2.0) * xLoc.Y()
                       - totalNotchWidth / 2.0 * yLoc.Y(),
        zEntry - in.slot_depth_mm);
    const gp_Ax2 notchAx(notchOrigin, gp_Dir(0, 0, 1), xLoc);
    const TopoDS_Shape keyNotch = pr::box(
        notchAx, in.key_tab_width_mm, totalNotchWidth,
        in.slot_depth_mm + kOverhang);

    // Fuse everything → single cut.
    TopoDS_Shape cutter = pr::fuse(slot, keyNotch);
    for (const auto& f : fingers) cutter = pr::fuse(cutter, f);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",     *entryId },
        { "start_x_mm",        in.start_x_mm },
        { "start_y_mm",        in.start_y_mm },
        { "end_x_mm",          in.end_x_mm },
        { "end_y_mm",          in.end_y_mm },
        { "axis_dir",          { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "slot_width_mm",     in.slot_width_mm },
        { "slot_depth_mm",     in.slot_depth_mm },
        { "finger_count",      in.finger_count },
        { "pitch_mm",          in.pitch_mm },
        { "finger_dia_mm",     in.finger_dia_mm },
        { "finger_depth_mm",   in.finger_depth_mm },
        { "key_tab_width_mm",  in.key_tab_width_mm },
        { "key_tab_depth_mm",  in.key_tab_depth_mm },
        { "key_tab_offset_mm", in.key_tab_offset_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "subfeature_count", 3 },          // slot + finger array + key tab
        { "finger_count",     in.finger_count },
        { "pitch_mm",         in.pitch_mm },
        { "slot_length_mm",   slotLen },
        { "key_tab_offset_mm", in.key_tab_offset_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::min(in.slot_width_mm, in.finger_dia_mm);
    tooling.tool_length_mm    = in.slot_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.02;
    const double rF = in.finger_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        slotLen * in.slot_width_mm * in.slot_depth_mm
      + in.finger_count * M_PI * rF * rF * in.finger_depth_mm
      + in.key_tab_width_mm * (2.0 * in.key_tab_depth_mm) * in.slot_depth_mm;
    tooling.est_cycle_time_s = std::max(5.0,
        slotLen * 0.05 + in.finger_count * 0.3 + 2.0);
    tooling.extra = {
        { "standard",       "JEDEC MO-189 / DIMM card-edge family" },
        { "operation_note", "Mill main slot, plunge N contact reliefs at pitch, mill key tab" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::pcb_card_edge_socket applied: {} fingers @ {} pitch faces {}->{}",
                  in.finger_count, in.pitch_mm,
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

    // Geometric heuristic: count small-radius vertical cylinders.  A
    // card-edge socket leaves N coplanar finger pockets — distinctive
    // signature is "many small-radius vertical cyls in a co-linear row".
    struct Cyl { double radius; gp_Pnt loc; };
    std::vector<Cyl> verticalCyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(d.Z()) > 0.9)
            verticalCyls.push_back({ c.Radius(), c.Axis().Location() });
    }
    if (verticalCyls.size() < 3) return out;

    // Find the most common small radius (tolerance 0.1 mm).
    std::sort(verticalCyls.begin(), verticalCyls.end(),
              [](const Cyl& a, const Cyl& b){ return a.radius < b.radius; });
    int bestCount = 0;
    double bestRadius = verticalCyls.front().radius;
    {
        size_t runStart = 0;
        for (size_t i = 1; i <= verticalCyls.size(); ++i) {
            if (i == verticalCyls.size() ||
                verticalCyls[i].radius - verticalCyls[runStart].radius > 0.15) {
                const int n = static_cast<int>(i - runStart);
                if (n > bestCount) { bestCount = n; bestRadius = verticalCyls[runStart].radius; }
                runStart = i;
            }
        }
    }
    if (bestCount < 3) return out;

    // Collect the matching cylinders and check colinearity / pitch.
    std::vector<gp_Pnt> pts;
    for (const auto& vc : verticalCyls) {
        if (std::abs(vc.radius - bestRadius) < 0.15) pts.push_back(vc.loc);
    }
    // Sort by X primarily, Y secondarily.
    std::sort(pts.begin(), pts.end(),
              [](const gp_Pnt& a, const gp_Pnt& b){
                  if (std::abs(a.X() - b.X()) > 1e-3) return a.X() < b.X();
                  return a.Y() < b.Y();
              });
    if (pts.size() < 3) return out;
    const double dx = pts.back().X() - pts.front().X();
    const double dy = pts.back().Y() - pts.front().Y();
    const double pitch = (pts.size() > 1)
        ? std::hypot(dx, dy) / (pts.size() - 1) : 0.0;

    json recovered = {
        { "start_x_mm",       pts.front().X() },
        { "start_y_mm",       pts.front().Y() },
        { "end_x_mm",         pts.back().X() },
        { "end_y_mm",         pts.back().Y() },
        { "axis_dir",         { 0.0, 0.0, -1.0 } },
        { "finger_count",     static_cast<int>(pts.size()) },
        { "finger_dia_mm",    2.0 * bestRadius },
        { "pitch_mm",         pitch },
    };
    RecognizedFeature r;
    r.skill_id         = kSkillId;
    r.recovered_params = recovered;
    // Confidence rises with finger count.
    r.confidence       = std::min(0.95, 0.5 + 0.02 * pts.size());
    r.matched_geometry = {
        { "finger_count",  static_cast<int>(pts.size()) },
        { "pitch_mm",      pitch },
    };
    out.push_back(r);
    return out;
}

}  // namespace koocadcam::skill::pcb_card_edge_socket
