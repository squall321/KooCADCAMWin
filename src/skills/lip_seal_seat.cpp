// @lat: [[engine/skills#lip_seal_seat]]

#include "lip_seal_seat.hpp"

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::lip_seal_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Tunables ─────────────────────────────────────────────────────────────
namespace {

constexpr double kDustLipRadial   = 0.5;   // mm — clearance for dust lip
constexpr double kDustLipAxial    = 1.5;   // mm — dust groove width
constexpr double kNoseChamferDeg  = 15.0;  // 15° lead-in per DIN 3760
constexpr double kNoseChamferSize = 1.5;   // 1.5 mm chamfer (axial leg)

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shaft_id_mm < 1.0) {
        r.add("DFM-002", "error",
              "lip_seal_seat: shaft_id_mm " + std::to_string(in.shaft_id_mm) +
              " < 1.0 mm");
    }
    if (in.seal_od_mm <= in.shaft_id_mm + 2.0) {
        r.add("DFM-LIPSEAL-WALL", "error",
              "lip_seal_seat: seal_od " + std::to_string(in.seal_od_mm) +
              " must be > shaft_id + 2.0 mm (seal wall ≥ 1 mm radial)");
    }
    if (in.seal_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lip_seal_seat: seal_thickness_mm must be > 0");
    }
    if (in.bore_depth_mm <= in.seal_thickness_mm) {
        r.add("DFM-INPUT", "error",
              "lip_seal_seat: bore_depth_mm must be > seal_thickness_mm");
    }

    auto faceId = wp.resolve(in.body_face);
    if (!faceId) {
        r.add("DFM-INPUT", "error",
              "lip_seal_seat: body_face datum unresolved");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lip_seal_seat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.body_face);
    if (!faceId) throw SkillError("lip_seal_seat: body_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir   = in.axis_dir;
    const double overhang = 0.05;
    double entryZ;
    if (adir.Z() < 0) entryZ = zMax + overhang;
    else              entryZ = zMin - overhang;

    const double shaftR = in.shaft_id_mm / 2.0;
    const double sealR  = in.seal_od_mm  / 2.0;

    const gp_Pnt origin(in.center_x_mm, in.center_y_mm, entryZ);
    const gp_Ax2 axBase(origin, adir);

    // ── Sub-feature 1: SHAFT BORE — full-depth cylindrical bore ─────────
    const TopoDS_Shape shaftBoreTool = pr::cylinder(
        axBase, shaftR, in.bore_depth_mm + overhang);

    // ── Sub-feature 2: LIP-SEAL COUNTERBORE ──────────────────────────────
    //
    // Larger-diameter coaxial cylinder at the entry, depth = seal_thickness.
    const TopoDS_Shape counterboreTool = pr::cylinder(
        axBase, sealR, in.seal_thickness_mm + overhang);

    // ── Sub-feature 3: DUST-LIP CLEARANCE GROOVE ────────────────────────
    //
    // Small annular relief at the BOTTOM of the counterbore: between shaft
    // bore (radius shaftR) and dust lip clearance radius (shaftR + 0.5 mm).
    // Located axially just past the seal_thickness depth.  Tool: annular
    // ring of inner_R = shaftR + ε, outer_R = shaftR + kDustLipRadial,
    // height = kDustLipAxial, positioned with its top at seal_thickness.
    //
    // Construct the annular ring in WORLD frame: the dust groove is between
    // axial planes z_top = entryZ + adir.Z()·seal_thickness (i.e. the bottom
    // of the counterbore) and z_bot = z_top + adir.Z()·kDustLipAxial.
    //
    // Use pr::annularRing(gp_Ax2(P, axisV), outerR, innerR, height) — the
    // ring extends "height" along axisV from P.
    const double dustOuterR = shaftR + kDustLipRadial;
    const double dustInnerR = shaftR;
    const gp_Pnt dustTop(in.center_x_mm,
                         in.center_y_mm,
                         entryZ + adir.Z() * in.seal_thickness_mm);
    const gp_Ax2 axDust(dustTop, adir);
    const TopoDS_Shape dustLipGroove = pr::annularRing(
        axDust, dustOuterR, dustInnerR, kDustLipAxial);

    // ── Sub-feature 4: NOSE CHAMFER (15° lead-in at body_face) ──────────
    //
    // Conical chamfer at the counterbore OD entry: r1 (at entryZ - chamfer
    // direction) = sealR + kNoseChamferSize·tan(15°), r2 (at chamfer depth
    // = kNoseChamferSize) = sealR.  Axis along adir.
    const double noseRise = kNoseChamferSize * std::tan(kNoseChamferDeg * M_PI / 180.0);
    const TopoDS_Shape noseChamfer = pr::annularConeRing(
        axBase,
        /*outerR1Bottom=*/ sealR + noseRise,
        /*outerR2Top  =*/ sealR,
        /*innerR      =*/ std::max(1e-3, sealR - 1e-3),
        /*height      =*/ kNoseChamferSize + overhang);

    // ── Fuse all tools then single boolean cut ───────────────────────────
    //
    // Concentric overlapping cylinders + ring + chamfer: fuse_first_then_cut
    // pattern (counterbore.cpp).
    const TopoDS_Shape fused1   = pr::fuse(shaftBoreTool, counterboreTool);
    const TopoDS_Shape fused2   = pr::fuse(fused1, dustLipGroove);
    const TopoDS_Shape fusedAll = pr::fuse(fused2, noseChamfer);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedAll);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "body_face_resolved", *faceId },
        { "center_x_mm",        in.center_x_mm },
        { "center_y_mm",        in.center_y_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "shaft_id_mm",        in.shaft_id_mm },
        { "seal_od_mm",         in.seal_od_mm },
        { "seal_thickness_mm",  in.seal_thickness_mm },
        { "bore_depth_mm",      in.bore_depth_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     4 },
        { "shaft_id_mm",          in.shaft_id_mm },
        { "seal_od_mm",           in.seal_od_mm },
        { "seal_thickness_mm",    in.seal_thickness_mm },
        { "bore_depth_mm",        in.bore_depth_mm },
        { "dust_lip_radial_mm",   kDustLipRadial },
        { "dust_lip_axial_mm",    kDustLipAxial },
        { "nose_chamfer_deg",     kNoseChamferDeg },
        { "nose_chamfer_size_mm", kNoseChamferSize },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;counterbore;grooving_insert;chamfer_tool";
    tooling.tool_dia_mm       = in.seal_od_mm;
    tooling.tool_length_mm    = in.bore_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 =
        M_PI * shaftR * shaftR * in.bore_depth_mm +
        M_PI * (sealR * sealR - shaftR * shaftR) * in.seal_thickness_mm +
        M_PI * (dustOuterR * dustOuterR - dustInnerR * dustInnerR) * kDustLipAxial;
    tooling.est_cycle_time_s  = std::max(5.0, in.bore_depth_mm / 20.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "kind", "shaft_bore" },
              { "diameter_mm", in.shaft_id_mm },
              { "depth_mm", in.bore_depth_mm } },
            { { "kind", "lip_seal_counterbore" },
              { "diameter_mm", in.seal_od_mm },
              { "depth_mm", in.seal_thickness_mm } },
            { { "kind", "dust_lip_groove" },
              { "radial_mm", kDustLipRadial },
              { "axial_mm",  kDustLipAxial } },
            { { "kind", "nose_chamfer" },
              { "angle_deg", kNoseChamferDeg },
              { "size_mm",   kNoseChamferSize } },
        } },
        { "standard", "ISO 6194-1 / DIN 3760 / SAE J518" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::lip_seal_seat applied: shaft={} seal={}×{} depth={}",
                  in.shaft_id_mm, in.seal_od_mm, in.seal_thickness_mm,
                  in.bore_depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = feat.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source", "metadata_replay" },
            { "pattern", feat.pattern },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::lip_seal_seat
