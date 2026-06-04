// @lat: [[engine/skills#panel_fastener_dzus_pocket]]

#include "panel_fastener_dzus_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::panel_fastener_dzus_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

bool isStandardStudDia(double d) {
    return std::abs(d - 6.35) < 1e-2 ||
           std::abs(d - 9.5)  < 1e-2;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "panel_fastener_dzus_pocket: face_id out of range");
    }
    if (!(in.stud_dia_mm > 0.0) ||
        !(in.receptacle_pocket_dia_mm > 0.0) ||
        !(in.receptacle_depth_mm > 0.0) ||
        !(in.key_slot_width_mm > 0.0) ||
        !(in.key_slot_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "panel_fastener_dzus_pocket: all dimensions must be > 0");
        return r;
    }
    if (!isStandardStudDia(in.stud_dia_mm)) {
        r.add("DFM-STUD-DIA", "error",
              "panel_fastener_dzus_pocket: stud_dia " +
              std::to_string(in.stud_dia_mm) +
              " mm not in MS27983 set {6.35, 9.5}");
    }
    if (in.receptacle_pocket_dia_mm < in.stud_dia_mm + 1.5) {
        r.add("DFM-POCKET-WIDTH", "error",
              "panel_fastener_dzus_pocket: pocket_dia must exceed stud_dia by ≥ 1.5 mm");
    }
    if (in.receptacle_depth_mm > 8.0) {
        r.add("DFM-POCKET-DEPTH", "error",
              "panel_fastener_dzus_pocket: receptacle_depth " +
              std::to_string(in.receptacle_depth_mm) +
              " mm > 8 mm (off-spec)");
    }
    if (in.key_slot_width_mm >= in.receptacle_pocket_dia_mm) {
        r.add("DFM-KEY-SLOT", "error",
              "panel_fastener_dzus_pocket: key_slot_width must be < pocket_dia");
    }
    if (in.key_slot_length_mm <= in.stud_dia_mm) {
        r.add("DFM-KEY-SLOT", "error",
              "panel_fastener_dzus_pocket: key_slot_length must exceed stud_dia");
    }
    if (in.key_slot_length_mm > in.receptacle_pocket_dia_mm * 1.5) {
        r.add("DFM-KEY-SLOT", "error",
              "panel_fastener_dzus_pocket: key_slot_length > 1.5 × pocket_dia");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "panel_fastener_dzus_pocket DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    constexpr double kOverhang = 0.05;
    const gp_Pnt entryPlanePoint(in.center_x_mm, in.center_y_mm, faceC.Z());
    const gp_Pnt toolStart(
        entryPlanePoint.X() + n.X() * kOverhang,
        entryPlanePoint.Y() + n.Y() * kOverhang,
        entryPlanePoint.Z() + n.Z() * kOverhang);

    // ── Sub-feature 1: stud through-bore ───────────────────────────────
    const double studR  = in.stud_dia_mm / 2.0;
    const double studHt = bboxDiag + 2.0 * kOverhang;
    const gp_Ax2 studAx(toolStart, drillDir);
    const TopoDS_Shape studTool = pr::cylinder(studAx, studR, studHt);

    // ── Sub-feature 2: receptacle pocket (concentric wider counterbore) ─
    const double pocketR  = in.receptacle_pocket_dia_mm / 2.0;
    const double pocketHt = in.receptacle_depth_mm + kOverhang;
    const gp_Ax2 pocketAx(toolStart, drillDir);
    const TopoDS_Shape pocketTool = pr::cylinder(pocketAx, pocketR, pocketHt);

    // ── Sub-feature 3: two key slots, 90° apart on entry face ──────────
    // Build first slot centered on stud axis, length along +X (face plane),
    // width along +Y, depth = receptacle_depth, going INTO body (drillDir).
    // gp_Ax2(P, V_axial, V_x): we want V_axial = drillDir (depth), V_x =
    // tangent in-plane.  Choose X axis of face plane as gp::DX().  Since
    // n may not be ±Z, we synthesize an in-plane X by orthogonalizing
    // gp::DX() against n.
    gp_Dir slotXDir(1.0, 0.0, 0.0);
    // If n is parallel to X (rare for top-face usage), use Y as fallback.
    if (std::abs(n.X()) > 0.9) {
        slotXDir = gp_Dir(0.0, 1.0, 0.0);
    }
    // Project slotXDir onto plane perpendicular to n: subtract n-component.
    const double dotXn = slotXDir.X() * n.X() + slotXDir.Y() * n.Y() + slotXDir.Z() * n.Z();
    const gp_Vec slotXVec(slotXDir.X() - dotXn * n.X(),
                          slotXDir.Y() - dotXn * n.Y(),
                          slotXDir.Z() - dotXn * n.Z());
    const gp_Dir slotXFinal(slotXVec);

    // Slot footprint: length × width centered on entry point.
    const double L = in.key_slot_length_mm;
    const double W = in.key_slot_width_mm;
    const double D = in.receptacle_depth_mm + kOverhang;

    // Origin = entry point shifted by -L/2 along slotX and -W/2 along (slotX × n)
    // gp_Ax2(origin, drillDir, slotX) → DX along slotX, DY along (drillDir × slotX),
    // DZ along drillDir.  We want DX = L (length), DY = W (width), DZ = D (depth).
    const gp_Vec slotYVec = gp_Vec(drillDir).Crossed(gp_Vec(slotXFinal));
    const gp_Dir slotYFinal(slotYVec);

    const gp_Pnt slot1Origin(
        entryPlanePoint.X() - 0.5 * L * slotXFinal.X() - 0.5 * W * slotYFinal.X() + n.X() * kOverhang,
        entryPlanePoint.Y() - 0.5 * L * slotXFinal.Y() - 0.5 * W * slotYFinal.Y() + n.Y() * kOverhang,
        entryPlanePoint.Z() - 0.5 * L * slotXFinal.Z() - 0.5 * W * slotYFinal.Z() + n.Z() * kOverhang);
    const gp_Ax2 slot1Ax(slot1Origin, drillDir, slotXFinal);
    const TopoDS_Shape slot1Tool = pr::box(slot1Ax, L, W, D);

    // Second slot is the first one rotated 90° about face normal (through entry point).
    const gp_Ax1 rotAxis(entryPlanePoint, n);
    gp_Trsf rot;
    rot.SetRotation(rotAxis, M_PI / 2.0);
    BRepBuilderAPI_Transform xform(slot1Tool, rot, true);
    if (!xform.IsDone()) {
        throw SkillError("panel_fastener_dzus_pocket: slot rotation failed");
    }
    const TopoDS_Shape slot2Tool = xform.Shape();

    // ── Sequential pr::cut loop (slice-14 guard) ────────────────────────
    TopoDS_Shape current = wp.shape();
    current = pr::cut(current, studTool);     // 1. stud bore
    current = pr::cut(current, pocketTool);   // 2. wider pocket (concentric)
    current = pr::cut(current, slot1Tool);    // 3a. key slot 1
    current = pr::cut(current, slot2Tool);    // 3b. key slot 2 (90° apart)
    const TopoDS_Shape newShape = current;

    // ── Derived volume ──────────────────────────────────────────────────
    const double studVol   = M_PI * studR * studR * (zMax - zMin);
    const double pocketVol = M_PI * (pocketR * pocketR - studR * studR) *
                             in.receptacle_depth_mm;
    const double slotVol   = L * W * in.receptacle_depth_mm;
    const double removedVol = studVol + pocketVol + 2.0 * slotVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",                   in.face_id },
        { "center_x_mm",               in.center_x_mm },
        { "center_y_mm",               in.center_y_mm },
        { "stud_dia_mm",               in.stud_dia_mm },
        { "receptacle_pocket_dia_mm",  in.receptacle_pocket_dia_mm },
        { "receptacle_depth_mm",       in.receptacle_depth_mm },
        { "key_slot_width_mm",         in.key_slot_width_mm },
        { "key_slot_length_mm",        in.key_slot_length_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "aerospace_feature_type",     "dzus_quarter_turn_receptacle" },
        { "subfeature_count",           4 },
        { "fastener_dia",               in.stud_dia_mm },
        { "receptacle_pocket_dia_mm",   in.receptacle_pocket_dia_mm },
        { "receptacle_depth_mm",        in.receptacle_depth_mm },
        { "key_slot_width_mm",          in.key_slot_width_mm },
        { "key_slot_length_mm",         in.key_slot_length_mm },
        { "key_slot_count",             2 },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "MS27983 / MS27984" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill";
    tooling.tool_dia_mm       = in.receptacle_pocket_dia_mm;
    tooling.tool_length_mm    = bboxDiag + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(3.0, in.receptacle_depth_mm / 25.0);
    tooling.extra = {
        { "aerospace_feature_type", "dzus_quarter_turn_receptacle" },
        { "fastener_family",        "Dzus" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::panel_fastener_dzus_pocket applied: stud={} pocket={} depth={}",
                  in.stud_dia_mm, in.receptacle_pocket_dia_mm, in.receptacle_depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

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
            { "source",                 "metadata_replay" },
            { "is_compound",            true },
            { "aerospace_feature_type", "dzus_quarter_turn_receptacle" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::panel_fastener_dzus_pocket
