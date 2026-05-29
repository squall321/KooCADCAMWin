// @lat: [[engine/skills#pivot_pin_clevis_compound]]

#include "pivot_pin_clevis_compound.hpp"

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

namespace koocadcam::skill::pivot_pin_clevis_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec table (MIL-S-5556 clevis fittings) ──────────────────────────────

namespace {

constexpr std::array<ClevisSpec, 4> kClevisTable {{
    { "CL-04", 4.0,  4.0, 3.0, 1.0,  5.0 },
    { "CL-06", 6.0,  5.0, 4.0, 1.2,  7.0 },
    { "CL-08", 8.0,  6.0, 5.0, 1.6, 10.0 },
    { "CL-10", 10.0, 8.0, 6.0, 2.0, 12.0 },
}};

}  // namespace

const ClevisSpec* lookupClevisSpec(const std::string& key)
{
    for (const auto& s : kClevisTable) {
        if (key == s.key) return &s;
    }
    return nullptr;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const ClevisSpec* spec = lookupClevisSpec(in.clevis_size);
    if (!spec) {
        r.add("DFM-SPEC", "error",
              "pivot_pin_clevis_compound: unknown clevis_size '" +
              in.clevis_size + "' (expected CL-04|CL-06|CL-08|CL-10)");
        return r;
    }

    if (in.u_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pivot_pin_clevis_compound: u_depth_mm must be > 0");
    }

    // MIL-S-5556: ear thickness must be at least pin_dia × 0.5 for shear.
    if (spec->ear_thk_mm < spec->pin_dia_mm * 0.5) {
        r.add("DFM-EAR-THK", "error",
              "pivot_pin_clevis_compound: ear_thk " +
              std::to_string(spec->ear_thk_mm) + " mm < pin_dia × 0.5 " +
              std::to_string(spec->pin_dia_mm * 0.5) + " mm (shear strength)");
    }

    if (spec->cotter_slot_mm < 0.8) {
        r.add("DFM-002", "error",
              "pivot_pin_clevis_compound: cotter_slot " +
              std::to_string(spec->cotter_slot_mm) + " mm < 0.8 mm");
    }
    if (spec->pin_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "pivot_pin_clevis_compound: pin_dia " +
              std::to_string(spec->pin_dia_mm) + " mm < 0.8 mm");
    }

    if (in.u_depth_mm < spec->pin_dia_mm) {
        r.add("DFM-INPUT", "warning",
              "pivot_pin_clevis_compound: u_depth less than pin_dia — "
              "pin would protrude beyond pocket floor");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pivot_pin_clevis_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ClevisSpec* spec = lookupClevisSpec(in.clevis_size);
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("pivot_pin_clevis_compound: entry_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax-xMin)*(xMax-xMin) + (yMax-yMin)*(yMax-yMin) + (zMax-zMin)*(zMax-zMin));
    (void)bboxDiag;
    const double kOverhang = 0.05;

    // SUBFEATURE 1: U-POCKET — rectangular slot through the lug.
    //
    // U opens against entry_face (assumed +Z normal in slice 1).  We model
    // the U-pocket as a centered box of u_width × ear_span × (u_depth + overhang).
    // ear_span = pin spans both ears, so transverse length = u_width × 3.
    const double earSpan = spec->u_width_mm * 3.0;
    const gp_Pnt uOrigin(
        in.center_x_mm - spec->u_width_mm / 2.0,
        in.center_y_mm - earSpan         / 2.0,
        zMax - in.u_depth_mm);
    const gp_Ax2 uAx(uOrigin, gp::DZ(), gp::DX());
    const TopoDS_Shape uPocket = pr::box(
        uAx, spec->u_width_mm, earSpan, in.u_depth_mm + kOverhang);

    // SUBFEATURE 2: CROSS-PIN BORE — perpendicular cylinder spanning both
    // ears.  Axis = pin_axis (default +Y).  Goes through entire ear set,
    // i.e. length = earSpan + overhang.
    //
    // The pin sits centered on (center_x, center_y, zMax - u_depth/2).
    const gp_Pnt pinCenter(in.center_x_mm,
                           in.center_y_mm - earSpan/2.0 - kOverhang,
                           zMax - in.u_depth_mm / 2.0);
    const gp_Ax2 pinAx(pinCenter, in.pin_axis, gp::DX());
    const TopoDS_Shape pinBore = pr::cylinder(
        pinAx, spec->pin_dia_mm / 2.0, earSpan + 2.0 * kOverhang);

    // SUBFEATURE 3: COTTER SLOT — narrow transverse slot intersecting the
    // pin axis on one side, perpendicular to both pin axis and u axis.
    // Modelled as a thin box.
    const double cotterLen = spec->pin_dia_mm * 2.0;
    const gp_Pnt cotterOrigin(
        in.center_x_mm - cotterLen          / 2.0,
        in.center_y_mm + earSpan/4.0 - spec->cotter_slot_mm / 2.0,
        zMax - in.u_depth_mm / 2.0 - spec->cotter_slot_mm   / 2.0);
    const gp_Ax2 cotterAx(cotterOrigin, gp::DZ(), gp::DX());
    const TopoDS_Shape cotterSlot = pr::box(
        cotterAx, cotterLen, spec->cotter_slot_mm, spec->cotter_slot_mm);

    // SUBFEATURE 4: BUSHING SEAT — concentric counterbore on the pin bore
    // at the +Y face.  Larger-diameter shallow cylinder, fused-then-cut for
    // robustness (per Constraints).
    const double seatDepth = std::max(0.5, spec->ear_thk_mm * 0.3);
    const gp_Pnt seatStart(in.center_x_mm,
                           in.center_y_mm + earSpan/2.0 + kOverhang,
                           zMax - in.u_depth_mm / 2.0);
    // Seat axis points INWARD (along -pin_axis from the outer face).
    gp_Dir seatDir(-in.pin_axis.X(), -in.pin_axis.Y(), -in.pin_axis.Z());
    const gp_Ax2 seatAx(seatStart, seatDir, gp::DX());
    const TopoDS_Shape seatTool = pr::cylinder(
        seatAx, spec->seat_dia_mm / 2.0, seatDepth);

    // Boolean composition:
    //   workpiece' = workpiece − (uPocket ∪ pinBore ∪ cotterSlot ∪ seatTool)
    //
    // For overlapping concentric cylinders (pinBore + seatTool), fuse first
    // per OCCT robustness guidance.
    const TopoDS_Shape pinFused  = pr::fuse(pinBore, seatTool);
    const TopoDS_Shape cutter1   = pr::fuse(uPocket, pinFused);
    const TopoDS_Shape cutter    = pr::fuse(cutter1, cotterSlot);
    const TopoDS_Shape newShape  = pr::cut(wp.shape(), cutter);

    // Build signature.
    json params = {
        { "entry_face_id", *entryId },
        { "center_x_mm",   in.center_x_mm },
        { "center_y_mm",   in.center_y_mm },
        { "u_axis",        { in.u_axis.X(), in.u_axis.Y(), in.u_axis.Z() } },
        { "pin_axis",      { in.pin_axis.X(), in.pin_axis.Y(), in.pin_axis.Z() } },
        { "u_depth_mm",    in.u_depth_mm },
        { "clevis_size",   in.clevis_size },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     4 },
        { "clevis_size",          in.clevis_size },
        { "u_width_mm",           spec->u_width_mm },
        { "pin_dia_mm",           spec->pin_dia_mm },
        { "ear_thk_mm",           spec->ear_thk_mm },
        { "cotter_slot_mm",       spec->cotter_slot_mm },
        { "seat_dia_mm",          spec->seat_dia_mm },
        { "u_depth_mm",           in.u_depth_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type = "end_mill;drill;tap";
    tooling.tool_dia_mm = spec->pin_dia_mm;
    tooling.tool_length_mm = in.u_depth_mm + 10.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 3;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double rPin = spec->pin_dia_mm / 2.0;
    const double rSeat = spec->seat_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        spec->u_width_mm * earSpan * in.u_depth_mm +
        M_PI * rPin * rPin * earSpan +
        cotterLen * spec->cotter_slot_mm * spec->cotter_slot_mm +
        M_PI * (rSeat * rSeat - rPin * rPin) * seatDepth;
    tooling.est_cycle_time_s = std::max(8.0, in.u_depth_mm * 0.8 + earSpan * 0.3);
    tooling.extra = {
        { "spec_table_ref", "MIL-S-5556 clevis fittings (ear shear 0.5×pin_dia)" },
        { "compound_chain", {
            { { "step", "u_pocket_cut" }, { "tool", "end_mill" } },
            { { "step", "pin_bore_cut" }, { "tool", "drill" } },
            { { "step", "cotter_slot" }, { "tool", "slotting_cutter" } },
            { { "step", "bushing_seat" }, { "tool", "counterbore" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::pivot_pin_clevis_compound applied: clevis={} u_depth={}",
                  in.clevis_size, in.u_depth_mm);

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

    // Geometric heuristic: count cylindrical faces.  A clevis has a
    // pin-bore cylinder + a bushing seat cylinder (counterbore step) AND
    // a U-pocket bottom plane.  Inventory:
    int cylCount = 0;
    double cylDiaMin = 1e9, cylDiaMax = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        ++cylCount;
        BRepAdaptor_Surface s(wp.face(i));
        const double d = 2.0 * s.Cylinder().Radius();
        cylDiaMin = std::min(cylDiaMin, d);
        cylDiaMax = std::max(cylDiaMax, d);
    }
    if (cylCount < 2) return out;

    // Match best spec by pin_dia ≈ cylDiaMin
    const ClevisSpec* best = nullptr;
    double bestErr = 1e9;
    for (const auto& s : kClevisTable) {
        const double err = std::abs(s.pin_dia_mm - cylDiaMin);
        if (err < bestErr) { bestErr = err; best = &s; }
    }
    if (!best || bestErr > 1.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    json recovered = {
        { "center_x_mm",   (xMin + xMax) / 2.0 },
        { "center_y_mm",   (yMin + yMax) / 2.0 },
        { "u_axis",        { 0.0, 0.0, -1.0 } },
        { "pin_axis",      { 0.0, 1.0,  0.0 } },
        { "u_depth_mm",    std::max(1.0, (zMax - zMin) * 0.6) },
        { "clevis_size",   std::string(best->key) },
    };
    json matched = {
        { "cyl_face_count", cylCount },
        { "cyl_dia_min_mm", cylDiaMin },
        { "cyl_dia_max_mm", cylDiaMax },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    return out;
}

}  // namespace koocadcam::skill::pivot_pin_clevis_compound
