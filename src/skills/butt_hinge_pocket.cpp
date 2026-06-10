// @lat: [[engine/skills#butt_hinge_pocket]]

#include "butt_hinge_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::butt_hinge_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── DFM gates ────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hinge_length_mm <= 0.0 || in.hinge_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "butt_hinge_pocket: hinge_length_mm and hinge_width_mm must be > 0");
        return r;
    }
    if (in.leaf_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "butt_hinge_pocket: leaf_thickness_mm must be > 0");
    }
    // Catalog range — typical butt hinges (residential & light hardware).
    if (in.hinge_length_mm < 30.0 || in.hinge_length_mm > 150.0) {
        r.add("DFM-HINGE-LENGTH", "error",
              "butt_hinge_pocket: hinge_length_mm " +
              std::to_string(in.hinge_length_mm) +
              " outside catalog range [30, 150] mm");
    }
    if (in.hinge_width_mm < 15.0 || in.hinge_width_mm > 60.0) {
        r.add("DFM-HINGE-WIDTH", "error",
              "butt_hinge_pocket: hinge_width_mm " +
              std::to_string(in.hinge_width_mm) +
              " outside catalog range [15, 60] mm");
    }
    if (in.leaf_thickness_mm < 0.8 || in.leaf_thickness_mm > 5.0) {
        r.add("DFM-HINGE-LEAF", "warning",
              "butt_hinge_pocket: leaf_thickness_mm " +
              std::to_string(in.leaf_thickness_mm) +
              " outside typical [0.8, 5.0] mm");
    }

    // Frame coverage — pocket must fit inside the workpiece (rough check
    // via bounding box).  This catches "drill into edge" mistakes early.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double halfL = in.hinge_length_mm / 2.0;
    const double halfW = in.hinge_width_mm  / 2.0;
    if (in.center_x_mm - halfL < xMin - 1e-3 ||
        in.center_x_mm + halfL > xMax + 1e-3 ||
        in.center_y_mm - halfW < yMin - 1e-3 ||
        in.center_y_mm + halfW > yMax + 1e-3)
    {
        r.add("DFM-HINGE-FRAME", "error",
              "butt_hinge_pocket: pocket footprint extends beyond stock bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Standardised screw geometry (M3.5 / #6 wood).
constexpr double kScrewPilotDia = 3.5;
constexpr double kScrewSeatDia  = 7.5;
constexpr double kScrewSeatDepth= 1.8;
constexpr double kScrewPilotDepth = 6.0;

// Leaf-clearance chamfer prism: 0.6 mm × 0.6 mm triangular relief along
// the inboard long edge of the pocket.  Modelled as a small box (slight
// over-build; CAM treats this as a 45° chamfer pass).
constexpr double kChamferSize = 0.6;

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "butt_hinge_pocket DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;
    constexpr double kOverhang = 0.05;

    const double halfL = in.hinge_length_mm / 2.0;
    const double halfW = in.hinge_width_mm  / 2.0;

    // ── Sub-feature 1: rectangular leaf pocket ─────────────────────────
    const gp_Pnt pocketOrigin(in.center_x_mm - halfL,
                              in.center_y_mm - halfW,
                              zTop - in.leaf_thickness_mm);
    const TopoDS_Shape pocketTool = pr::box(
        gp_Ax2(pocketOrigin, gp::DZ()),
        in.hinge_length_mm, in.hinge_width_mm,
        in.leaf_thickness_mm + kOverhang);

    // ── Sub-feature 2: two screw counterbores at 1/4 and 3/4 of length ─
    // Each is a fused (pilot + seat) cutter, à la counterbore.cpp pattern.
    const double screw1X = in.center_x_mm - halfL * 0.5;
    const double screw2X = in.center_x_mm + halfL * 0.5;
    const double screwY  = in.center_y_mm;   // along centerline of pocket

    auto makeScrewTool = [&](double sx, double sy) {
        const gp_Pnt p(sx, sy, zTop + kOverhang);
        const gp_Ax2 ax(p, gp_Dir(0, 0, -1));
        const TopoDS_Shape pilot = pr::cylinder(
            ax, kScrewPilotDia / 2.0, kScrewPilotDepth + kOverhang);
        const TopoDS_Shape seat  = pr::cylinder(
            ax, kScrewSeatDia  / 2.0, kScrewSeatDepth  + kOverhang);
        return pr::fuse(pilot, seat);
    };
    const TopoDS_Shape screw1 = makeScrewTool(screw1X, screwY);
    const TopoDS_Shape screw2 = makeScrewTool(screw2X, screwY);

    // ── Sub-feature 3: leaf-clearance chamfer prism along inboard edge ─
    // We model it as a slim box on the +Y inboard edge of the pocket.
    const gp_Pnt chamferOrigin(
        in.center_x_mm - halfL,
        in.center_y_mm + halfW - kChamferSize,
        zTop - kChamferSize);
    const TopoDS_Shape chamferTool = pr::box(
        gp_Ax2(chamferOrigin, gp::DZ()),
        in.hinge_length_mm, kChamferSize, kChamferSize + kOverhang);

    // ── Single fused cutter → single cut ───────────────────────────────
    const TopoDS_Shape fused1 = pr::fuse(pocketTool, screw1);
    const TopoDS_Shape fused2 = pr::fuse(fused1,     screw2);
    const TopoDS_Shape fusedCutter = pr::fuse(fused2, chamferTool);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ──────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",       in.center_x_mm },
        { "center_y_mm",       in.center_y_mm },
        { "axis_dir",          { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "hinge_length_mm",   in.hinge_length_mm },
        { "hinge_width_mm",    in.hinge_width_mm },
        { "leaf_thickness_mm", in.leaf_thickness_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "subfeature_count",       4 },   // pocket + 2 screws + chamfer
        { "hinge_length_mm",        in.hinge_length_mm },
        { "hinge_width_mm",         in.hinge_width_mm },
        { "leaf_thickness_mm",      in.leaf_thickness_mm },
        { "screw_pilot_dia_mm",     kScrewPilotDia },
        { "screw_seat_dia_mm",      kScrewSeatDia },
        { "screw_positions",        json::array({
            json::array({ screw1X, screwY }),
            json::array({ screw2X, screwY }),
        }) },
        { "chamfer_size_mm",        kChamferSize },
        { "axis_dir",               { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "end_mill;counterbore;chamfer_mill";
    tooling.tool_dia_mm = kScrewSeatDia;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 2;
    // Stock-removal estimate (sum of sub-feature volumes, ignoring small overlap).
    const double pocketV = in.hinge_length_mm * in.hinge_width_mm * in.leaf_thickness_mm;
    const double seatR   = kScrewSeatDia  / 2.0;
    const double pilotR  = kScrewPilotDia / 2.0;
    const double oneScrewV =
        M_PI * pilotR * pilotR * kScrewPilotDepth +
        M_PI * (seatR * seatR - pilotR * pilotR) * kScrewSeatDepth;
    const double chamferV = in.hinge_length_mm * kChamferSize * kChamferSize * 0.5;
    tooling.stock_removed_mm3 = pocketV + 2.0 * oneScrewV + chamferV;
    tooling.est_cycle_time_s  = std::max(2.0, pocketV / 200.0);
    tooling.extra = {
        { "tool_sequence", json::array({
            { { "tool_type", "end_mill" }, { "purpose", "leaf_pocket" },
              { "tool_dia_mm", 6.0 }, { "depth_mm", in.leaf_thickness_mm } },
            { { "tool_type", "counterbore" }, { "purpose", "screw_seat" },
              { "tool_dia_mm", kScrewSeatDia }, { "pass_count", 2 } },
            { { "tool_type", "chamfer_mill" }, { "purpose", "leaf_clearance" },
              { "chamfer_size_mm", kChamferSize } },
        }) },
        { "standard", "DIN-1052-residential-butt-hinge" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::butt_hinge_pocket applied: {}x{}x{} + 2 screws + chamfer",
                  in.hinge_length_mm, in.hinge_width_mm, in.leaf_thickness_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognize: metadata replay only ─────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::butt_hinge_pocket
