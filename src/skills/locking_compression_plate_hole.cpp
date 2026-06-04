// @lat: [[engine/skills#locking_compression_plate_hole]]

#include "locking_compression_plate_hole.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::locking_compression_plate_hole {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.locking_thread_dia_mm <= 0.0) {
        r.add("DFM-LCP-INPUT", "error",
              "locking_compression_plate_hole: locking_thread_dia_mm must be > 0");
        return r;
    }
    if (in.dynamic_compression_slot_length_mm <
        in.locking_thread_dia_mm * 1.5) {
        r.add("DFM-LCP-SLOT", "error",
              "locking_compression_plate_hole: slot length " +
              std::to_string(in.dynamic_compression_slot_length_mm) +
              " mm < 1.5 × locking_thread_dia (" +
              std::to_string(in.locking_thread_dia_mm * 1.5) +
              " mm) — no DCP travel room");
    }
    if (in.plate_thickness_mm < 0.5 * in.locking_thread_dia_mm) {
        r.add("DFM-LCP-THICK", "error",
              "locking_compression_plate_hole: plate_thickness " +
              std::to_string(in.plate_thickness_mm) +
              " mm < 0.5 × locking_thread_dia (" +
              std::to_string(0.5 * in.locking_thread_dia_mm) +
              " mm) — insufficient thread engagement");
    }
    if (in.locking_thread_pitch_mm < 0.4 ||
        in.locking_thread_pitch_mm > 2.0) {
        r.add("DFM-LCP-PITCH", "error",
              "locking_compression_plate_hole: pitch " +
              std::to_string(in.locking_thread_pitch_mm) +
              " mm outside ISO bone-screw envelope [0.4, 2.0]");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "locking_compression_plate_hole DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("locking_compression_plate_hole: entry_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir = in.axis_dir;
    const gp_Dir sdir = in.slot_dir;        // long axis of the slot
    // Slot has half-cylinder ends with radius = locking_thread_dia / 2.
    const double slotR    = in.locking_thread_dia_mm / 2.0;
    const double slotLen  = in.dynamic_compression_slot_length_mm;
    const double rectLen  = std::max(0.0, slotLen - 2.0 * slotR);

    const double cutDepth = in.plate_thickness_mm + 0.5;
    const double kOverhang = 0.05;

    // Reference top point (entry plane in ±Z-axis fast path).
    gp_Pnt topCenter(in.position_x_mm, in.position_y_mm, (zMin + zMax) / 2.0);
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        const double entryZ = (adir.Z() < 0) ? zMax : zMin;
        topCenter = gp_Pnt(in.position_x_mm, in.position_y_mm, entryZ);
    }

    // Sub-feature 1a: central rectangular slot body (if rectLen > 0).
    // Perpendicular to sdir, in the entry plane.
    const gp_Dir perp(-sdir.Y(), sdir.X(), 0.0);

    std::vector<TopoDS_Shape> slotTools;
    if (rectLen > 1e-6) {
        const gp_Pnt boxOrigin(
            topCenter.X() - 0.5 * rectLen * sdir.X() - slotR * perp.X(),
            topCenter.Y() - 0.5 * rectLen * sdir.Y() - slotR * perp.Y(),
            topCenter.Z() + adir.Z() * kOverhang);   // +0.05 above for Z=-1
        const gp_Ax2 boxAx(boxOrigin, adir, sdir);
        slotTools.push_back(
            pr::box(boxAx, rectLen, 2.0 * slotR, cutDepth));
    }

    // Sub-feature 1b: two end half-cylinders (full cylinders cover both halves
    // of the stadium and overlap with the rectangle — fuse cleans up).
    const gp_Pnt endA(
        topCenter.X() - 0.5 * rectLen * sdir.X(),
        topCenter.Y() - 0.5 * rectLen * sdir.Y(),
        topCenter.Z() + adir.Z() * kOverhang);
    const gp_Pnt endB(
        topCenter.X() + 0.5 * rectLen * sdir.X(),
        topCenter.Y() + 0.5 * rectLen * sdir.Y(),
        topCenter.Z() + adir.Z() * kOverhang);
    slotTools.push_back(pr::cylinder(gp_Ax2(endA, adir), slotR, cutDepth));
    slotTools.push_back(pr::cylinder(gp_Ax2(endB, adir), slotR, cutDepth));

    // Fuse the stadium body together.
    TopoDS_Shape slotFused = slotTools[0];
    for (size_t i = 1; i < slotTools.size(); ++i)
        slotFused = pr::fuse(slotFused, slotTools[i]);

    // Sub-feature 2: locking-thread bore at one end of the slot.
    // Coincides with endB cylinder; modelled as bore of locking_thread_dia
    // (smooth cylinder — actual thread carved by tap_thread skill later).
    const TopoDS_Shape lockingTool =
        pr::cylinder(gp_Ax2(endB, adir),
                     in.locking_thread_dia_mm / 2.0,
                     cutDepth);

    // Fuse with slot for single cut.
    const TopoDS_Shape fused    = pr::fuse(slotFused, lockingTool);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // Signature
    json params = {
        { "entry_face_id",      *entryId },
        { "position_x_mm",      in.position_x_mm },
        { "position_y_mm",      in.position_y_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "slot_dir",           { sdir.X(), sdir.Y(), sdir.Z() } },
        { "dynamic_compression_slot_length_mm",
          in.dynamic_compression_slot_length_mm },
        { "locking_thread_dia_mm",   in.locking_thread_dia_mm },
        { "locking_thread_pitch_mm", in.locking_thread_pitch_mm },
        { "plate_thickness_mm",      in.plate_thickness_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    2 },
        { "medical_feature_type","lcp_combi_hole" },
        { "screw_size",          in.locking_thread_dia_mm },
        { "slot_length_mm",      in.dynamic_compression_slot_length_mm },
        { "slot_width_mm",       in.locking_thread_dia_mm },
        { "locking_thread_dia_mm",   in.locking_thread_dia_mm },
        { "locking_thread_pitch_mm", in.locking_thread_pitch_mm },
        { "locking_end_position", "+slot_dir" },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "slot_dir",            { sdir.X(), sdir.Y(), sdir.Z() } },
        { "implant_standard",    "AO_LCP" },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "endmill;tap";
    tooling.tool_dia_mm = in.locking_thread_dia_mm;
    tooling.tool_length_mm = in.plate_thickness_mm * 2.0 + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 4;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double slotArea = std::max(0.0, rectLen) * (2.0 * slotR)
                          + M_PI * slotR * slotR;        // stadium area
    tooling.stock_removed_mm3 = slotArea * in.plate_thickness_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.plate_thickness_mm / 15.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::locking_compression_plate_hole applied: slot {} thread M{} pitch {}",
                  in.dynamic_compression_slot_length_mm,
                  in.locking_thread_dia_mm,
                  in.locking_thread_pitch_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true },
                                { "medical_feature_type",
                                  "lcp_combi_hole" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::locking_compression_plate_hole
