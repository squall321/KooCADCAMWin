// @lat: [[engine/skills#linear_bushing_seat]]

#include "linear_bushing_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::linear_bushing_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr double kShoulderOffsetMm = 4.0;   // shoulder ID = bushing_od - 2 × 4 = OD-8
constexpr double kGrooveWidthMm    = 1.5;
constexpr double kGrooveDepthMm    = 0.5;
constexpr double kShoulderThickMm  = 2.0;   // axial thickness of retention shoulder

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bushing_od_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "linear_bushing_seat: bushing_od_mm must be > 0");
    }
    if (in.bushing_l_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "linear_bushing_seat: bushing_l_mm must be > 0");
    }

    // Retention shoulder inner-diameter check.
    const double shoulderID = in.bushing_od_mm - 2.0 * kShoulderOffsetMm;
    if (shoulderID < 0.8) {
        r.add("DFM-002", "error",
              "linear_bushing_seat: retention shoulder ID " +
              std::to_string(shoulderID) +
              " mm < min 0.8 mm (bushing_od must be > " +
              std::to_string(2.0 * kShoulderOffsetMm + 0.8) + " mm)");
    }

    // Shoulder wall thickness check (DFM-LB-SHOULDER): (OD-ID)/2 >= 1 mm.
    if (in.bushing_od_mm > 0.0 && (in.bushing_od_mm - shoulderID) / 2.0 < 1.0) {
        r.add("DFM-LB-SHOULDER", "error",
              "linear_bushing_seat: shoulder wall thickness " +
              std::to_string((in.bushing_od_mm - shoulderID)/2.0) +
              " mm < 1 mm");
    }

    // Groove feasibility: bushing must be long enough for 2 grooves at
    // 25% and 75% of bushing length, each kGrooveWidthMm wide.
    if (in.bushing_l_mm > 0.0 && in.bushing_l_mm < 6.0) {
        r.add("DFM-LB-GROOVE", "error",
              "linear_bushing_seat: bushing_l_mm " +
              std::to_string(in.bushing_l_mm) +
              " < 6 mm (cannot fit 2 lubrication grooves with adequate spacing)");
    }

    (void)wp;
    (void)kShoulderThickMm;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "linear_bushing_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("linear_bushing_seat: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kOverhang = 0.05;
    const gp_Pnt    axisLoc = in.axis.Location();
    const gp_Dir    axisDir = in.axis.Direction();

    // The bushing OD bore extends from axisLoc in the +axisDir direction for
    // bushing_l_mm.  We keep the math axis-agnostic by parametrising along
    // axisDir.

    // ── Sub-feature 1: bushing OD bore (cylinder) ──────────────────────
    const double bushingR = in.bushing_od_mm / 2.0;
    // Start a tad before the seat opening for a clean entry cut.
    const gp_Pnt boreStart(
        axisLoc.X() - axisDir.X() * kOverhang,
        axisLoc.Y() - axisDir.Y() * kOverhang,
        axisLoc.Z() - axisDir.Z() * kOverhang);
    const gp_Ax2 boreAx(boreStart, axisDir);
    const TopoDS_Shape boreTool =
        pr::cylinder(boreAx, bushingR, in.bushing_l_mm + kOverhang);

    // ── Sub-feature 2: retention shoulder thru-bore (smaller dia) ─────
    // Goes BEYOND the bushing seat to actually act as a thru-hole shoulder.
    // The shoulder is the annular material between bushingR and shoulderR
    // at the bottom of the seat; we cut a smaller-radius cylinder all the
    // way through, so the bottom of the seat shows a smaller hole = shoulder.
    const double shoulderR = bushingR - kShoulderOffsetMm;
    // Extend the small thru-bore far enough to definitely exit the part.
    const double bboxDiag = std::sqrt(
        (xMax-xMin)*(xMax-xMin) + (yMax-yMin)*(yMax-yMin) + (zMax-zMin)*(zMax-zMin));
    const double shoulderToolLen = in.bushing_l_mm + bboxDiag + 2.0 * kOverhang;
    const TopoDS_Shape shoulderTool =
        pr::cylinder(boreAx, shoulderR, shoulderToolLen);

    // Concentric overlapping cylinders → fuse first then cut.
    const TopoDS_Shape concentricCutter = pr::fuse(boreTool, shoulderTool);

    // ── Sub-features 3, 4: 2 lubrication grooves (annular ring slots) ──
    // Each groove is a short annular ring at axial offsets 0.25*L and 0.75*L
    // along the bushing axis.  The ring's outer radius extends INTO the
    // material (bushingR + kGrooveDepthMm) by kGrooveDepthMm, inner radius =
    // bushingR.  Cutting this annular ring carves the groove into the bore wall.
    auto makeGroove = [&](double axialOffset) -> TopoDS_Shape {
        const gp_Pnt gStart(
            axisLoc.X() + axisDir.X() * (axialOffset - kGrooveWidthMm / 2.0),
            axisLoc.Y() + axisDir.Y() * (axialOffset - kGrooveWidthMm / 2.0),
            axisLoc.Z() + axisDir.Z() * (axialOffset - kGrooveWidthMm / 2.0));
        const gp_Ax2 gAx(gStart, axisDir);
        return pr::annularRing(gAx,
                               bushingR + kGrooveDepthMm,
                               bushingR,
                               kGrooveWidthMm);
    };
    const TopoDS_Shape groove1 = makeGroove(in.bushing_l_mm * 0.25);
    const TopoDS_Shape groove2 = makeGroove(in.bushing_l_mm * 0.75);

    // Cut sequence: concentric (bore + shoulder thru-bore) + 2 grooves.
    const std::vector<TopoDS_Shape> allTools = {
        concentricCutter, groove1, groove2
    };
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), allTools);

    constexpr int kSubFeatures = 4;   // bore + shoulder thru + 2 grooves

    json params = {
        { "entry_face_id",  *entryId },
        { "axis_location",  { axisLoc.X(), axisLoc.Y(), axisLoc.Z() } },
        { "axis_direction", { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "bushing_od_mm",  in.bushing_od_mm },
        { "bushing_l_mm",   in.bushing_l_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      kSubFeatures },
        { "bore_count",            1 },
        { "shoulder_count",        1 },
        { "lub_groove_count",      2 },
        { "bushing_od_mm",         in.bushing_od_mm },
        { "bushing_l_mm",          in.bushing_l_mm },
        { "shoulder_id_mm",        2.0 * shoulderR },
        { "shoulder_offset_mm",    kShoulderOffsetMm },
        { "groove_width_mm",       kGrooveWidthMm },
        { "groove_depth_mm",       kGrooveDepthMm },
        { "groove_positions_mm",   { in.bushing_l_mm * 0.25, in.bushing_l_mm * 0.75 } },
        { "axis_direction",        { axisDir.X(), axisDir.Y(), axisDir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "boring_bar;drill;groove_tool";
    tooling.tool_dia_mm      = in.bushing_od_mm;
    tooling.tool_length_mm   = in.bushing_l_mm * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double boreVol     = M_PI * bushingR * bushingR * in.bushing_l_mm;
    const double shoulderVol = M_PI * shoulderR * shoulderR * kShoulderThickMm;
    const double grooveVol   = 2.0 * M_PI *
        ((bushingR + kGrooveDepthMm)*(bushingR + kGrooveDepthMm) - bushingR*bushingR) *
        kGrooveWidthMm;
    tooling.stock_removed_mm3 = boreVol + shoulderVol + grooveVol;
    tooling.est_cycle_time_s  = std::max(5.0, in.bushing_l_mm * 0.4 + 2.0 * 1.5);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" },
              { "purpose",   "bushing OD bore" },
              { "tool_dia_mm", in.bushing_od_mm },
              { "depth_mm",    in.bushing_l_mm } },
            { { "tool_type", "drill" },
              { "purpose",   "retention shoulder thru-bore" },
              { "tool_dia_mm", 2.0 * shoulderR } },
            { { "tool_type", "groove_tool" },
              { "purpose",   "2 lubrication grooves" },
              { "groove_width_mm", kGrooveWidthMm },
              { "groove_depth_mm", kGrooveDepthMm },
              { "pass_count",  2 } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::linear_bushing_seat applied: od={} l={} shoulder_id={}",
                  in.bushing_od_mm, in.bushing_l_mm, 2.0 * shoulderR);

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
            { "source",            "metadata_replay" },
            { "subfeature_count",  f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::linear_bushing_seat
