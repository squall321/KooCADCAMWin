// @lat: [[engine/skills#isolator_grommet_seat]]

#include "isolator_grommet_seat.hpp"

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

namespace koocadcam::skill::isolator_grommet_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Isolator spec table ──────────────────────────────────────────────────
//
// Vendor-agnostic vibration isolator sizes (SAE J2627 anti-vibration
// isolators).  cb = counterbore.

namespace {

struct IsolatorSpec {
    const char* size;
    double cb_dia_mm;
    double cb_depth_mm;
    double groove_depth_mm;     // radial depth of ID groove
    double groove_width_mm;     // axial width
    double groove_z_offset_mm;  // measured from entry plane
    double notch_w_mm;
    double notch_h_mm;          // along radial direction at pocket bottom
    double notch_d_mm;          // along axial (depth)
};

constexpr std::array<IsolatorSpec, 4> kIsoTable {{
    { "ISO-S",   8.0,  3.0, 0.5, 0.6, 1.0, 1.0, 0.6, 0.5 },
    { "ISO-M",  12.0,  5.0, 0.7, 0.8, 1.5, 1.5, 0.8, 0.7 },
    { "ISO-L",  16.0,  7.0, 0.9, 1.0, 2.0, 2.0, 1.0, 0.9 },
    { "ISO-XL", 22.0,  9.0, 1.2, 1.4, 3.0, 2.5, 1.2, 1.1 },
}};

const IsolatorSpec* findSpec(const std::string& size)
{
    for (const auto& g : kIsoTable) {
        if (size == g.size) return &g;
    }
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const IsolatorSpec* sp = findSpec(in.isolator_size);
    if (!sp) {
        r.add("DFM-CONN-SIZE", "error",
              "isolator_grommet_seat: unknown isolator_size '" +
              in.isolator_size +
              "' (expected ISO-S/ISO-M/ISO-L/ISO-XL)");
        return r;
    }

    if (sp->cb_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "isolator_grommet_seat: counterbore Ø " +
              std::to_string(sp->cb_dia_mm) + " < 0.8 mm");
    }
    if (sp->groove_width_mm < 0.5) {
        r.add("DFM-GROOVE-WIDTH", "error",
              "isolator_grommet_seat: groove width " +
              std::to_string(sp->groove_width_mm) +
              " < 0.5 mm (form-grind clearance)");
    }
    if (sp->groove_z_offset_mm + sp->groove_width_mm / 2.0 >= sp->cb_depth_mm) {
        r.add("DFM-GROOVE-POSITION", "error",
              "isolator_grommet_seat: groove falls outside counterbore depth");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > 0.0 && thickness < sp->cb_depth_mm + 1.0) {
        r.add("DFM-CB-DEPTH", "error",
              "isolator_grommet_seat: workpiece thickness " +
              std::to_string(thickness) +
              " < cb_depth + 1 mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-cuts (fused before subtraction):
//   1. COUNTERBORE        — cylinder of cb_dia × cb_depth
//   2. ID RETENTION GROOVE — annular ring at the inside wall, between
//                            radii (cb_dia/2) and (cb_dia/2 + groove_depth)
//   3. ROTATION NOTCH     — small box keyway cut into the bottom of the
//                            counterbore (radial keyway).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "isolator_grommet_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const IsolatorSpec* sp = findSpec(in.isolator_size);
    const double cbR     = sp->cb_dia_mm / 2.0;
    const double grooveR = cbR + sp->groove_depth_mm;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    const double entryZ = (in.axis_dir.Z() < 0) ? zMax : zMin;
    const double signZ  = (in.axis_dir.Z() < 0) ? -1.0 : 1.0;

    // ── Sub-cut 1: counterbore ───────────────────────────────────────────
    const gp_Ax2 axCB(
        gp_Pnt(in.position_x_mm, in.position_y_mm,
               entryZ + signZ * kOverhang),
        in.axis_dir);
    const TopoDS_Shape cb = pr::cylinder(
        axCB, cbR, sp->cb_depth_mm + kOverhang);

    // ── Sub-cut 2: ID groove ─────────────────────────────────────────────
    const double grooveCenterZ = entryZ + signZ * sp->groove_z_offset_mm;
    const double grooveStartZ  = grooveCenterZ - sp->groove_width_mm / 2.0;
    const gp_Ax2 axGroove(
        gp_Pnt(in.position_x_mm, in.position_y_mm, grooveStartZ),
        gp::DZ());
    const TopoDS_Shape groove = pr::annularRing(
        axGroove, grooveR, cbR * 0.99, sp->groove_width_mm);

    // ── Sub-cut 3: rotation notch (rectangular box at pocket bottom) ─────
    const double notchBottomZ = entryZ + signZ * sp->cb_depth_mm;
    // Notch extends radially outward from cbR a tiny bit.
    const gp_Pnt notchOrigin(
        in.position_x_mm - sp->notch_w_mm / 2.0,
        in.position_y_mm + cbR - sp->notch_h_mm * 0.2,
        notchBottomZ - signZ * sp->notch_d_mm);
    const gp_Ax2 axNotch(notchOrigin, gp::DZ());
    const TopoDS_Shape notch = pr::box(
        axNotch, sp->notch_w_mm, sp->notch_h_mm, sp->notch_d_mm + kOverhang);

    // FUSE all cuts into one.
    TopoDS_Shape fusedCutter = pr::fuse(cb, groove);
    fusedCutter = pr::fuse(fusedCutter, notch);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "isolator_size",  in.isolator_size },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     3 },     // cb + groove + notch
        { "isolator_size",        in.isolator_size },
        { "cb_dia_mm",            sp->cb_dia_mm },
        { "cb_depth_mm",          sp->cb_depth_mm },
        { "groove_depth_mm",      sp->groove_depth_mm },
        { "groove_width_mm",      sp->groove_width_mm },
        { "groove_z_offset_mm",   sp->groove_z_offset_mm },
        { "notch_w_mm",           sp->notch_w_mm },
        { "axis_dir",             { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "endmill;groove_form;keyseat";
    tooling.tool_dia_mm      = sp->cb_dia_mm;
    tooling.tool_length_mm   = sp->cb_depth_mm + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 3;
    tooling.cutting_speed_sfm = 320.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 =
        M_PI * cbR * cbR * sp->cb_depth_mm +
        M_PI * (grooveR * grooveR - cbR * cbR) * sp->groove_width_mm +
        sp->notch_w_mm * sp->notch_h_mm * sp->notch_d_mm;
    tooling.est_cycle_time_s = std::max(3.0, sp->cb_depth_mm * 0.8);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" },     { "tool_dia_mm", sp->cb_dia_mm },
              { "depth_mm", sp->cb_depth_mm } },
            { { "tool_type", "groove_form" }, { "depth_mm", sp->groove_depth_mm },
              { "width_mm", sp->groove_width_mm } },
            { { "tool_type", "keyseat" },     { "w_mm", sp->notch_w_mm },
              { "d_mm", sp->notch_d_mm } },
        } },
        { "isolator_standard", "SAE_J2627" },
        { "isolator_size",     in.isolator_size },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::isolator_grommet_seat applied: {} at ({},{})",
                  in.isolator_size, in.position_x_mm, in.position_y_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata path
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric heuristic: cylindrical face matching one of the spec
    // counterbore diameters.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        const double dia = 2.0 * cyl.Radius();
        for (const auto& g : kIsoTable) {
            if (std::abs(dia - g.cb_dia_mm) < 0.1) {
                const gp_Pnt loc = cyl.Axis().Location();
                json rp = {
                    { "axis_dir",      json::array({ 0.0, 0.0, -1.0 }) },
                    { "position_x_mm", loc.X() },
                    { "position_y_mm", loc.Y() },
                    { "isolator_size", g.size },
                };
                RecognizedFeature rf;
                rf.skill_id         = kSkillId;
                rf.recovered_params = rp;
                rf.confidence       = 0.6;
                rf.matched_geometry = {
                    { "face_id",      i },
                    { "matched_size", g.size },
                    { "source",       "geometric" },
                };
                out.push_back(rf);
                break;
            }
        }
    }
    return out;
}

}  // namespace koocadcam::skill::isolator_grommet_seat
