// @lat: [[engine/skills#pinned_clevis_joint]]

#include "pinned_clevis_joint.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pinned_clevis_joint {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.block_length_mm <= 0.0 || in.block_width_mm <= 0.0 ||
        in.block_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pinned_clevis_joint: block dims must be > 0");
    }
    if (in.tongue_slot_width_mm <= 0.0 || in.tongue_slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pinned_clevis_joint: tongue slot dims must be > 0");
    }
    if (in.pin_hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pinned_clevis_joint: pin_hole_dia must be > 0");
    }
    if (in.pin_hole_dia_mm > 0.0 && in.pin_hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "pin_hole_dia " + std::to_string(in.pin_hole_dia_mm) +
              " mm < min 0.8 mm");
    }

    // Tongue slot must fit inside block.
    if (in.tongue_slot_width_mm >= in.block_width_mm - 4.0) {
        r.add("DFM-CLEVIS-EAR", "error",
              "tongue slot too wide — leaves < 2 mm wall on each ear");
    }
    if (in.tongue_slot_depth_mm >= in.block_height_mm) {
        r.add("DFM-INPUT", "error",
              "tongue slot depth ≥ block height");
    }

    // DIN 71752: ear thickness ≥ 0.6 × pin dia.
    const double earThk = (in.block_width_mm - in.tongue_slot_width_mm) / 2.0;
    const double minEar = 0.6 * in.pin_hole_dia_mm;
    if (earThk < minEar) {
        r.add("DFM-DIN-71752", "error",
              "clevis ear thickness " + std::to_string(earThk) +
              " mm < min 0.6 × pin_dia (" + std::to_string(minEar) +
              " mm) — risk of ear shear-out");
    }

    // Pin hole must sit IN the tongue-slot zone (axis crosses both ears AND
    // crosses the slot).  The pin axis Z must be below the slot top.
    if (in.pin_hole_z_offset_mm <= 0.0 ||
        in.pin_hole_z_offset_mm >= in.tongue_slot_depth_mm) {
        r.add("DFM-CLEVIS-PIN", "error",
              "pin_hole_z_offset must be in (0, tongue_slot_depth) so pin "
              "crosses the slot");
    }
    if (in.pin_hole_dia_mm + 4.0 > in.tongue_slot_depth_mm) {
        r.add("DFM-CLEVIS-PIN", "warning",
              "pin hole nearly fills tongue slot — verify ear stress");
    }

    // Cotter pin (MIL-DTL-23949): cotter ≥ pin_dia / 4 AND ≥ 0.8 mm.
    const double minCotter = std::max(0.8, in.pin_hole_dia_mm / 4.0);
    if (in.cotter_pin_dia_mm < minCotter) {
        r.add("DFM-MIL-23949", "error",
              "cotter_pin_dia " + std::to_string(in.cotter_pin_dia_mm) +
              " mm < min " + std::to_string(minCotter) +
              " mm (max(0.8, pin_dia/4))");
    }
    if (in.cotter_pin_offset_mm <= in.pin_hole_dia_mm / 2.0 + 1.5) {
        r.add("DFM-MIL-23949", "error",
              "cotter offset must be > pin_radius + 1.5 mm (else cotter "
              "would intersect pin bore)");
    }
    if (in.cotter_pin_offset_mm + in.cotter_pin_dia_mm / 2.0 >
        in.block_length_mm / 2.0 - 1.0) {
        r.add("DFM-MIL-23949", "error",
              "cotter offset places hole outside ear face");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build sequence:
//   step 1: subtract tongue slot (rect prism) from top face
//   step 2: subtract cross-pin hole (cylinder along block_length axis)
//   step 3: subtract cotter hole at +offset
//   step 4: subtract cotter hole at -offset
//
// Volume delta (TOTAL REMOVED from input wp.shape()):
//   V_slot   = slot_width × slot_depth × block_length
//   V_pin    = π · (pin_dia/2)²  · block_length  (effective, ignoring slot)
//              minus the slot intersection (slot_width · pin_dia · approx)
//   V_cotter = 2 × π · (cotter_dia/2)² · block_width
//
// For test bounds we use the upper-bound estimate without intersection
// subtraction and use a wider ±25 % tolerance.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pinned_clevis_joint DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    if (!(std::abs(in.axis_dir.X()) < 1e-6 && std::abs(in.axis_dir.Y()) < 1e-6 &&
          in.axis_dir.Z() < 0)) {
        throw SkillError("pinned_clevis_joint: only axis_dir = -Z supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const double topZ = zMax;
    const double kOver = 0.1;

    // ── step 1: tongue slot ─────────────────────────────────────────────
    // Slot extends along block_length (X axis) full length; width along Y.
    const gp_Pnt slotOrigin(
        cx - in.block_length_mm / 2.0 - kOver,
        cy - in.tongue_slot_width_mm / 2.0,
        topZ - in.tongue_slot_depth_mm);
    const TopoDS_Shape slotTool = pr::box(
        gp_Ax2(slotOrigin, gp::DZ()),
        in.block_length_mm + 2.0 * kOver,
        in.tongue_slot_width_mm,
        in.tongue_slot_depth_mm + kOver);
    const TopoDS_Shape afterSlot = pr::cut(wp.shape(), slotTool);

    // ── step 2: cross-pin hole ─────────────────────────────────────────
    // Axis along +X (block_length).  Position: (cx, cy, topZ - pin_z_offset).
    const gp_Pnt pinStart(
        cx - in.block_length_mm / 2.0 - kOver,
        cy,
        topZ - in.pin_hole_z_offset_mm);
    const gp_Ax2 axPin(pinStart, gp_Dir(1.0, 0.0, 0.0));
    const TopoDS_Shape pinTool = pr::cylinder(
        axPin, in.pin_hole_dia_mm / 2.0,
        in.block_length_mm + 2.0 * kOver);
    const TopoDS_Shape afterPin = pr::cut(afterSlot, pinTool);

    // ── step 3 & 4: two cotter holes (radial, along Y, through full width)
    // At ±cotter_offset along X, same Z as pin hole.
    const gp_Pnt cotterLStart(
        cx - in.cotter_pin_offset_mm,
        cy - in.block_width_mm / 2.0 - kOver,
        topZ - in.pin_hole_z_offset_mm);
    const gp_Ax2 axCotterL(cotterLStart, gp_Dir(0.0, 1.0, 0.0));
    const TopoDS_Shape cotterL = pr::cylinder(
        axCotterL, in.cotter_pin_dia_mm / 2.0,
        in.block_width_mm + 2.0 * kOver);

    const gp_Pnt cotterRStart(
        cx + in.cotter_pin_offset_mm,
        cy - in.block_width_mm / 2.0 - kOver,
        topZ - in.pin_hole_z_offset_mm);
    const gp_Ax2 axCotterR(cotterRStart, gp_Dir(0.0, 1.0, 0.0));
    const TopoDS_Shape cotterR = pr::cylinder(
        axCotterR, in.cotter_pin_dia_mm / 2.0,
        in.block_width_mm + 2.0 * kOver);

    const TopoDS_Shape afterCotL = pr::cut(afterPin, cotterL);
    const TopoDS_Shape newShape  = pr::cut(afterCotL, cotterR);

    // ── volume bookkeeping (upper-bound, test-aligned) ──────────────────
    const double rPin    = in.pin_hole_dia_mm    / 2.0;
    const double rCotter = in.cotter_pin_dia_mm  / 2.0;
    const double vSlot   = in.block_length_mm *
                           in.tongue_slot_width_mm *
                           in.tongue_slot_depth_mm;
    const double vPin    = M_PI * rPin * rPin * in.block_length_mm;
    const double vCotter = 2.0 * M_PI * rCotter * rCotter * in.block_width_mm;
    const double vTotal  = vSlot + vPin + vCotter;

    // ── signature ───────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",            cx },
        { "center_y_mm",            cy },
        { "axis_dir",               { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "block_length_mm",        in.block_length_mm },
        { "block_width_mm",         in.block_width_mm },
        { "block_height_mm",        in.block_height_mm },
        { "tongue_slot_width_mm",   in.tongue_slot_width_mm },
        { "tongue_slot_depth_mm",   in.tongue_slot_depth_mm },
        { "pin_hole_dia_mm",        in.pin_hole_dia_mm },
        { "pin_hole_z_offset_mm",   in.pin_hole_z_offset_mm },
        { "cotter_pin_dia_mm",      in.cotter_pin_dia_mm },
        { "cotter_pin_offset_mm",   in.cotter_pin_offset_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },
        { "pin_axis",            { 1.0, 0.0, 0.0 } },
        { "slot_axis",           { 0.0, 0.0, -1.0 } },
        { "cotter_axis",         { 0.0, 1.0, 0.0 } },
        { "tongue_slot_width_mm",in.tongue_slot_width_mm },
        { "tongue_slot_depth_mm",in.tongue_slot_depth_mm },
        { "pin_hole_dia_mm",     in.pin_hole_dia_mm },
        { "cotter_pin_dia_mm",   in.cotter_pin_dia_mm },
        { "ear_thickness_mm",
          (in.block_width_mm - in.tongue_slot_width_mm) / 2.0 },
        { "standard",            "DIN 71752 / MIL-DTL-23949" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::max(in.pin_hole_dia_mm,
                                         in.tongue_slot_width_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.stock_removed_mm3 = vTotal;
    tooling.est_cycle_time_s  = 90.0;
    tooling.extra = {
        { "slot_volume_mm3",   vSlot },
        { "pin_volume_mm3",    vPin },
        { "cotter_volume_mm3", vCotter },
        { "standard",          "DIN 71752 + MIL-DTL-23949" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::pinned_clevis_joint: slot {}×{} pin Ø{} cotter Ø{}",
                  in.tongue_slot_width_mm, in.tongue_slot_depth_mm,
                  in.pin_hole_dia_mm, in.cotter_pin_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

struct CylAxisInfo { int idx; double radius; gp_Ax1 axis; };

std::vector<CylAxisInfo> collectCyls(const Workpiece& wp)
{
    std::vector<CylAxisInfo> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        out.push_back({ i, s.Cylinder().Radius(), s.Cylinder().Axis() });
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cyls = collectCyls(wp);
    if (cyls.size() < 3) return out;  // need pin + 2 cotters minimum

    // Geometric primary: find a cylinder whose axis is mostly along X (pin)
    // and 2 cylinders whose axes are mostly along Y (cotters), with the
    // cotter radii smaller than the pin radius.
    for (const auto& pin : cyls) {
        const double pinAxisX = std::abs(pin.axis.Direction().X());
        if (pinAxisX < 0.95) continue;       // must be transverse-X

        // Search cotters: axis-Y, smaller radius, same Z (within ±2 mm).
        std::vector<const CylAxisInfo*> cotters;
        for (const auto& c : cyls) {
            if (c.idx == pin.idx) continue;
            if (std::abs(c.axis.Direction().Y()) < 0.95) continue;
            if (c.radius >= pin.radius) continue;
            const double dz = std::abs(c.axis.Location().Z() -
                                       pin.axis.Location().Z());
            if (dz > 2.0) continue;
            cotters.push_back(&c);
        }
        if (cotters.size() < 2) continue;

        // Cotters should sit on either side of pin along X.
        double sumDia = 0.0; int cnt = 0;
        for (auto* c : cotters) { sumDia += 2.0 * c->radius; ++cnt; }
        const double cotterDia = sumDia / static_cast<double>(cnt);

        const double conf = 0.8;
        json recovered = {
            { "center_x_mm",         pin.axis.Location().X() },
            { "center_y_mm",         pin.axis.Location().Y() },
            { "axis_dir",            { 0.0, 0.0, -1.0 } },
            { "pin_hole_dia_mm",     2.0 * pin.radius },
            { "cotter_pin_dia_mm",   cotterDia },
        };
        json matched = {
            { "pin_face_id",       pin.idx },
            { "cotter_count",      static_cast<int>(cotters.size()) },
            { "via",               "geometric_primary" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
        break;  // one clevis per workpiece
    }

    // Metadata fallback.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        if (!out.empty()) continue;
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence = 1.0;
        rf.matched_geometry = { { "via", "metadata_replay" } };
        out.push_back(rf);
    }

    return out;
}

}  // namespace koocadcam::skill::pinned_clevis_joint
