// @lat: [[engine/skills#din_rail_mount_slot]]

#include "din_rail_mount_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::din_rail_mount_slot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// DIN EN 60715 standard rail dimensions.
namespace {
constexpr double kRcvW          = 35.0;  // receiver groove width (rail width)
constexpr double kRcvD          = 7.5;   // receiver groove depth (rail height)
constexpr double kFinW          = 27.0;  // bottom finger relief width
constexpr double kFinD          = 1.5;   // finger relief extra depth
constexpr double kHoleSpacing   = 25.0;  // retention hole spacing
constexpr double kEndMargin     = 10.0;  // first hole offset from each rail end
constexpr double kHoleDia       = 4.5;   // M4 clearance
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.rail_length_mm < 60.0) {
        r.add("DFM-DIN35", "error",
              "din_rail_mount_slot rail_length " +
              std::to_string(in.rail_length_mm) +
              " mm < 60 mm — too short to host 2 retention holes + flange");
    }
    if (in.rail_length_mm > 500.0) {
        r.add("DFM-DIN35", "error",
              "din_rail_mount_slot rail_length " +
              std::to_string(in.rail_length_mm) +
              " mm > 500 mm — single-piece milling impractical (split into sections)");
    }

    // Slice-1 limitation: rail_dir must be ±X or ±Y.
    const bool alongX = std::abs(std::abs(in.rail_dir.X()) - 1.0) < 1e-3
                        && std::abs(in.rail_dir.Y()) < 1e-3;
    const bool alongY = std::abs(std::abs(in.rail_dir.Y()) - 1.0) < 1e-3
                        && std::abs(in.rail_dir.X()) < 1e-3;
    if (!alongX && !alongY) {
        r.add("DFM-INPUT", "error",
              "din_rail_mount_slot: rail_dir must be ±X or ±Y in slice 1");
    }
    if (std::abs(in.rail_dir.Z()) > 1e-3) {
        r.add("DFM-INPUT", "error",
              "din_rail_mount_slot: rail_dir must be horizontal (Z=0)");
    }

    if (in.plate_t_mm < kRcvD + kFinD + 2.0) {
        r.add("DFM-DIN35", "error",
              "din_rail_mount_slot: plate_t " + std::to_string(in.plate_t_mm) +
              " mm < " + std::to_string(kRcvD + kFinD + 2.0) +
              " mm — plate too thin to hold receiver + relief + 2 mm floor");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "din_rail_mount_slot DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("din_rail_mount_slot: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("din_rail_mount_slot: only axis_dir = -Z supported in slice-1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Local axes — yLoc is perpendicular to rail_dir in plane.
    const gp_Dir xLoc(in.rail_dir.X(), in.rail_dir.Y(), 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    // ── 1) Top receiver groove (35 × 7.5 along rail) ──────────────────
    //
    // Box origin: at start - (kRcvW/2) along yLoc, at z = zMax - kRcvD.
    // Build with V = +Z so DZ extends up.  DX along xLoc (slot length),
    // DY along yLoc (rail width = kRcvW), DZ vertical = kRcvD + kOverhang.
    const gp_Pnt rcvOrigin(
        in.start_x_mm - kRcvW / 2.0 * yLoc.X(),
        in.start_y_mm - kRcvW / 2.0 * yLoc.Y(),
        zMax - kRcvD);
    const gp_Ax2 rcvAx(rcvOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape rcvBox = pr::box(rcvAx, in.rail_length_mm, kRcvW,
                                         kRcvD + kOverhang);

    // ── 2) Bottom spring-finger relief slot (27 × 1.5, BELOW receiver) ─
    const gp_Pnt finOrigin(
        in.start_x_mm - kFinW / 2.0 * yLoc.X(),
        in.start_y_mm - kFinW / 2.0 * yLoc.Y(),
        zMax - kRcvD - kFinD);
    const gp_Ax2 finAx(finOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape finBox = pr::box(finAx, in.rail_length_mm, kFinW,
                                         kFinD + kOverhang);

    // Fuse the two groove boxes — they share the bottom face of the
    // receiver as a common contact area (the receiver's bottom rectangle
    // is wider than the relief but coincident at z = zMax - kRcvD).  Use
    // fuse_first_then_cut.
    TopoDS_Shape fused;
    try {
        fused = pr::fuse(rcvBox, finBox);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("din_rail_mount_slot: groove fuse failed: ") + e.what());
    }

    // ── 3-4) Two M4 retention through holes ─────────────────────────────
    //
    // Place at center along yLoc, at the front (start + endMargin) and
    // back (start + rail_length - endMargin) ends of the rail.
    const double holeOffsetA = kEndMargin;
    const double holeOffsetB = in.rail_length_mm - kEndMargin;
    // Validate they don't overlap (rail_length should be ≥ 2 × kEndMargin + kHoleSpacing-min)
    if (holeOffsetB - holeOffsetA < kHoleSpacing) {
        throw SkillError(
            "din_rail_mount_slot: rail too short to host 2 retention holes at spacing");
    }

    auto holePt = [&](double offset) {
        return gp_Pnt(
            in.start_x_mm + offset * xLoc.X(),
            in.start_y_mm + offset * xLoc.Y(),
            zMax + kOverhang);
    };
    const gp_Ax2 hAx1(holePt(holeOffsetA), in.axis_dir);
    const gp_Ax2 hAx2(holePt(holeOffsetB), in.axis_dir);
    const double holeH = in.plate_t_mm + 2.0 * kOverhang;
    const TopoDS_Shape hole1 = pr::cylinder(hAx1, kHoleDia / 2.0, holeH);
    const TopoDS_Shape hole2 = pr::cylinder(hAx2, kHoleDia / 2.0, holeH);

    try {
        fused = pr::fuse(fused, hole1);
        fused = pr::fuse(fused, hole2);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("din_rail_mount_slot: hole fuse failed: ") + e.what());
    }

    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), fused);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("din_rail_mount_slot: cut failed: ") + e.what());
    }

    json params = {
        { "entry_face_id",   *entryId },
        { "start_x_mm",      in.start_x_mm },
        { "start_y_mm",      in.start_y_mm },
        { "rail_dir",        { in.rail_dir.X(), in.rail_dir.Y(), in.rail_dir.Z() } },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "rail_length_mm",  in.rail_length_mm },
        { "plate_t_mm",      in.plate_t_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },
        { "rail_length_mm",      in.rail_length_mm },
        { "receiver_w_mm",       kRcvW },
        { "receiver_d_mm",       kRcvD },
        { "finger_w_mm",         kFinW },
        { "finger_d_mm",         kFinD },
        { "hole_dia_mm",         kHoleDia },
        { "hole_offset_A_mm",    holeOffsetA },
        { "hole_offset_B_mm",    holeOffsetB },
        { "standard",            "DIN EN 60715 (top-hat 35x7.5)" },
        { "rail_dir",            { in.rail_dir.X(), in.rail_dir.Y(), in.rail_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type   = "endmill;drill";
    tooling.tool_dia_mm = kRcvW;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 4;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double rcvV = in.rail_length_mm * kRcvW * kRcvD;
    const double finV = in.rail_length_mm * kFinW * kFinD;
    const double holeV = 2.0 * M_PI * (kHoleDia/2.0) * (kHoleDia/2.0) * in.plate_t_mm;
    tooling.stock_removed_mm3 = rcvV + finV + holeV;
    tooling.est_cycle_time_s = std::max(8.0, in.rail_length_mm / 40.0 + 6.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" }, { "tool_dia_mm", kRcvW },
              { "depth_mm", kRcvD }, { "note", "receiver groove" } },
            { { "tool_type", "endmill" }, { "tool_dia_mm", kFinW },
              { "depth_mm", kFinD }, { "note", "finger relief slot" } },
            { { "tool_type", "drill" }, { "tool_dia_mm", kHoleDia },
              { "depth_mm", in.plate_t_mm }, { "pass_count", 2 },
              { "note", "M4 retention through holes" } },
        } },
        { "standard", "DIN EN 60715" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::din_rail_mount_slot applied: len={} plate_t={} faces {}→{}",
                  in.rail_length_mm, in.plate_t_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
// Metadata replay (compound parallel rectangular slots are not unique).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "feature_history_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::din_rail_mount_slot
