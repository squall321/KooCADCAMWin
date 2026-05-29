// @lat: [[engine/skills#gas_spring_clevis_pocket]]

#include "gas_spring_clevis_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::gas_spring_clevis_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// SAE J515 / DIN 71752 clevis dimensional gates.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.clevis_w <= 0.0 || in.clevis_d <= 0.0 || in.clevis_length <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gas_spring_clevis_pocket: clevis_w, clevis_d, clevis_length "
              "must all be > 0");
        return r;
    }
    if (in.pin_dia <= 0.0 || in.retention_groove_w <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gas_spring_clevis_pocket: pin_dia and retention_groove_w must "
              "be > 0");
        return r;
    }

    if (in.clevis_w < 6.0 || in.clevis_w > 40.0) {
        r.add("DFM-CLEVIS-RANGE", "error",
              "gas_spring_clevis_pocket: clevis_w " + std::to_string(in.clevis_w) +
              " mm outside Bansbach/Stabilus catalog [6, 40]");
    }
    if (in.clevis_d < in.clevis_w / 2.0) {
        r.add("DFM-CLEVIS-DEPTH", "error",
              "gas_spring_clevis_pocket: clevis_d " + std::to_string(in.clevis_d) +
              " < clevis_w/2 (" + std::to_string(in.clevis_w / 2.0) +
              ") — side walls too short for pin support");
    }

    if (in.pin_dia < 4.0) {
        r.add("DFM-CLEVIS-PIN", "error",
              "gas_spring_clevis_pocket: pin_dia " + std::to_string(in.pin_dia) +
              " mm < min M4 (4 mm) gas-spring eyelet pin");
    }
    if (in.pin_dia >= in.clevis_w / 2.0) {
        r.add("DFM-CLEVIS-PIN", "error",
              "gas_spring_clevis_pocket: pin_dia " + std::to_string(in.pin_dia) +
              " >= clevis_w/2 (" + std::to_string(in.clevis_w / 2.0) +
              ") — pin would weaken or break through side walls");
    }

    if (in.retention_groove_w < 0.4 || in.retention_groove_w > 2.0) {
        r.add("DFM-CLEVIS-GROOVE", "warning",
              "gas_spring_clevis_pocket: retention_groove_w " +
              std::to_string(in.retention_groove_w) +
              " mm outside e-clip standard range [0.4, 2.0]");
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double thickness = zMax - zMin;
        if (thickness < in.clevis_d + 2.0) {
            r.add("DFM-CLEVIS-WALL", "error",
                  "gas_spring_clevis_pocket: stock thickness " +
                  std::to_string(thickness) +
                  " < clevis_d + 2 mm wall (" +
                  std::to_string(in.clevis_d + 2.0) + ")");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain:
//   1. U-shape pocket box (rectangular cutter into face)
//   2. Cross pin through-hole (cylindrical cutter perpendicular to pocket axis)
//   3. Two retention grooves (small annular bands around the pin hole, on
//      each outer side of the part).  We collapse this to a single fused
//      multi-band cutter for the cut() call.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gas_spring_clevis_pocket DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError(
        "gas_spring_clevis_pocket: face_id datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    constexpr double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Sub-feature 1: pocket box.  Axis aligned (pocket length along +X,
    // width along +Y, depth along axis_dir).  Centered on (x, y).
    // For simplicity we assume axis_dir = -Z (the common case).
    const double topZ = zMax;
    const gp_Pnt boxOrigin(
        in.position_x_mm - in.clevis_length / 2.0,
        in.position_y_mm - in.clevis_w / 2.0,
        topZ - in.clevis_d);
    const gp_Ax2 boxAx(boxOrigin, gp::DZ());
    const TopoDS_Shape pocketTool = pr::box(
        boxAx, in.clevis_length, in.clevis_w, in.clevis_d + kOverhang);

    // Sub-feature 2: cross pin hole.  Goes through the part along +Y
    // (perpendicular to pocket axis & axis_dir).  Centered at
    // (x, y, topZ - clevis_d/2) and extends bbox-wide.
    const double pinZ = topZ - in.clevis_d / 2.0;
    const gp_Pnt pinStart(
        in.position_x_mm,
        yMin - kOverhang,
        pinZ);
    const gp_Ax2 pinAx(pinStart, gp_Dir(0.0, 1.0, 0.0));
    const double pinLength = (yMax - yMin) + 2.0 * kOverhang;
    const TopoDS_Shape pinTool = pr::cylinder(
        pinAx, in.pin_dia / 2.0, pinLength);

    // Sub-feature 3: two retention grooves — thin annular rings around
    // pin axis, just inside the part outer faces.  Outer radius = pin_dia/2 + 0.6,
    // inner radius = pin_dia/2.  Axial width = retention_groove_w.
    const double grooveOuterR = in.pin_dia / 2.0 + 0.6;
    const double grooveInnerR = in.pin_dia / 2.0;
    // Groove 1: near yMin side.
    const gp_Pnt g1Start(
        in.position_x_mm,
        yMin + 1.0,             // 1 mm in from face
        pinZ);
    const gp_Ax2 g1Ax(g1Start, gp_Dir(0.0, 1.0, 0.0));
    const TopoDS_Shape groove1 = pr::annularRing(
        g1Ax, grooveOuterR, grooveInnerR, in.retention_groove_w);
    // Groove 2: near yMax side.
    const gp_Pnt g2Start(
        in.position_x_mm,
        yMax - 1.0 - in.retention_groove_w,
        pinZ);
    const gp_Ax2 g2Ax(g2Start, gp_Dir(0.0, 1.0, 0.0));
    const TopoDS_Shape groove2 = pr::annularRing(
        g2Ax, grooveOuterR, grooveInnerR, in.retention_groove_w);

    // Compose: fuse pocket + pin + grooves.  Pocket and pin are not
    // concentric but they overlap inside the part — fuse handles it.
    TopoDS_Shape fused = pr::fuse(pocketTool, pinTool);
    fused = pr::fuse(fused, groove1);
    fused = pr::fuse(fused, groove2);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",            *faceId },
        { "position_x_mm",      in.position_x_mm },
        { "position_y_mm",      in.position_y_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "clevis_w",           in.clevis_w },
        { "clevis_d",           in.clevis_d },
        { "clevis_length",      in.clevis_length },
        { "pin_dia",            in.pin_dia },
        { "retention_groove_w", in.retention_groove_w },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           3 },
        { "subfeatures", json::array({
            { { "op", "u_pocket_box" },
              { "length_mm", in.clevis_length },
              { "width_mm",  in.clevis_w },
              { "depth_mm",  in.clevis_d } },
            { { "op", "cross_pin_through" },
              { "diameter_mm", in.pin_dia },
              { "axis",        "perpendicular" } },
            { { "op", "retention_grooves" },
              { "groove_count", 2 },
              { "width_mm",     in.retention_groove_w } },
        }) },
        { "clevis_w",                   in.clevis_w },
        { "clevis_d",                   in.clevis_d },
        { "clevis_length",              in.clevis_length },
        { "pin_dia",                    in.pin_dia },
        { "pin_axis_z_mm",              pinZ },
        { "retention_groove_w",         in.retention_groove_w },
        { "side_wall_count",            2 },
        { "pocket_bottom_present",      true },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "endmill;drill;groove_tool";
    tooling.tool_dia_mm       = in.clevis_w;
    tooling.tool_length_mm    = in.clevis_d * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pocketVol = in.clevis_length * in.clevis_w * in.clevis_d;
    const double pinVol    = M_PI * (in.pin_dia / 2.0) * (in.pin_dia / 2.0)
                           * (yMax - yMin);
    const double grooveVol = 2.0 * M_PI *
        (grooveOuterR * grooveOuterR - grooveInnerR * grooveInnerR) *
        in.retention_groove_w;
    tooling.stock_removed_mm3 = pocketVol + pinVol + grooveVol;
    tooling.est_cycle_time_s  = std::max(3.0, in.clevis_d / 25.0 + 2.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" },
              { "tool_dia_mm", in.clevis_w },
              { "depth_mm",    in.clevis_d } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.pin_dia } },
            { { "tool_type", "groove_tool" },
              { "width_mm",    in.retention_groove_w },
              { "pass_count",  2 } },
        } },
        { "feature_standard", "SAE J515 / DIN 71752 gas-spring clevis pocket" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::gas_spring_clevis_pocket applied: w={} d={} pin={}",
                  in.clevis_w, in.clevis_d, in.pin_dia);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source", "metadata_replay" },
            { "is_compound", true },
            { "subfeature_count", 3 },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::gas_spring_clevis_pocket
