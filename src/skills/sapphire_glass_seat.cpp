// @lat: [[engine/skills#sapphire_glass_seat]]

#include "sapphire_glass_seat.hpp"

#include "_as568_table.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::sapphire_glass_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.glass_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sapphire_glass_seat: glass_dia_mm must be > 0");
        return r;
    }
    if (in.glass_dia_mm < 10.0 || in.glass_dia_mm > 60.0) {
        r.add("DFM-GLASS-SIZE", "warning",
              "sapphire_glass_seat: glass_dia " +
              std::to_string(in.glass_dia_mm) +
              " mm outside typical [10, 60] watch range");
    }
    if (in.ledge_depth_mm < 1.0) {
        r.add("DFM-LEDGE-DEPTH", "error",
              "sapphire_glass_seat: ledge_depth " +
              std::to_string(in.ledge_depth_mm) +
              " mm < min 1.0 mm — insufficient glass support");
    }
    if (in.o_ring_cs_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sapphire_glass_seat: o_ring_cs_mm must be > 0");
    }
    // AS568 -004 family CS = 1.0 mm; we accept 0.7-2.0 with warning otherwise.
    // Cross-referenced against the central AS568 catalog (_as568_table.hpp):
    // the smallest dash sizes are -908 (CS = 1.27 mm) and the -006/-011/-016
    // family (CS = 1.78 mm), both of which fall inside [0.7, 2.0].
    static_assert(as568::kAs568.size() > 0,
                  "central AS568 catalog must be populated");
    if (in.o_ring_cs_mm < 0.7 || in.o_ring_cs_mm > 2.0) {
        r.add("DFM-ORING-CS", "warning",
              "sapphire_glass_seat: o_ring_cs " +
              std::to_string(in.o_ring_cs_mm) +
              " mm outside AS568 -004 family practical range");
    }

    // Derive inner bore Ø defaults.
    const double innerBoreDia = (in.inner_bore_dia_mm > 0.0)
        ? in.inner_bore_dia_mm
        : (in.glass_dia_mm - 1.0);
    const double innerBoreDepth = (in.inner_bore_depth_mm > 0.0)
        ? in.inner_bore_depth_mm
        : (in.ledge_depth_mm + 1.5);

    if (innerBoreDia >= in.glass_dia_mm) {
        r.add("DFM-LEDGE-GEOM", "error",
              "sapphire_glass_seat: inner_bore_dia " +
              std::to_string(innerBoreDia) +
              " mm ≥ glass_dia (" + std::to_string(in.glass_dia_mm) +
              ") — no ledge support");
    }
    if (innerBoreDia > in.glass_dia_mm - 0.8) {
        r.add("DFM-LEDGE-GEOM", "error",
              "sapphire_glass_seat: ledge width " +
              std::to_string((in.glass_dia_mm - innerBoreDia) / 2.0) +
              " mm < 0.4 mm (per side) — insufficient ledge support");
    }
    if (innerBoreDepth <= in.ledge_depth_mm) {
        r.add("DFM-LEDGE-GEOM", "error",
              "sapphire_glass_seat: inner_bore_depth must be > ledge_depth");
    }

    // Shoulder height = ledge_depth.  Groove fits in shoulder?
    const double grooveWidth = in.o_ring_cs_mm;
    if (grooveWidth + in.o_ring_z_offset_mm + 0.4 > in.ledge_depth_mm) {
        r.add("DFM-GROOVE-FIT", "error",
              "sapphire_glass_seat: groove width + offset (" +
              std::to_string(grooveWidth + in.o_ring_z_offset_mm) +
              ") + 0.4 mm margin > shoulder height (" +
              std::to_string(in.ledge_depth_mm) + ")");
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double thickness = zMax - zMin;
        if (in.ledge_depth_mm > thickness - 1.0) {
            r.add("DFM-LEDGE-DEPTH", "error",
                  "sapphire_glass_seat: ledge_depth " +
                  std::to_string(in.ledge_depth_mm) +
                  " mm > case_thickness − 1 mm (" +
                  std::to_string(thickness - 1.0) + ")");
        }
        if (innerBoreDepth > thickness - 0.5) {
            r.add("DFM-LEDGE-DEPTH", "error",
                  "sapphire_glass_seat: inner_bore_depth " +
                  std::to_string(innerBoreDepth) +
                  " mm exceeds available case_thickness");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain (3 cuts):
//   1. outer bore cut (sapphire OD slip-fit) to ledge_depth
//   2. inner bore cut (smaller dia) — concentric overlap with outer
//      → outer + inner FUSED then single cut (counterbore pattern)
//   3. O-ring side groove (annular ring on the shoulder vertical wall)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sapphire_glass_seat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double rOuter = in.glass_dia_mm / 2.0 + 0.05;   // slip-fit
    const double innerBoreDia = (in.inner_bore_dia_mm > 0.0)
        ? in.inner_bore_dia_mm : (in.glass_dia_mm - 1.0);
    const double innerBoreDepth = (in.inner_bore_depth_mm > 0.0)
        ? in.inner_bore_depth_mm : (in.ledge_depth_mm + 1.5);
    const double rInner = innerBoreDia / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    constexpr double kOverhang = 0.05;

    // ── Sub-feature 1: outer bore (sapphire slip-fit) ────────────────────
    const gp_Pnt outerStart(in.center_x_mm, in.center_y_mm, topZ + kOverhang);
    const gp_Ax2 outerAx(outerStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape outerBore = pr::cylinder(
        outerAx, rOuter, in.ledge_depth_mm + kOverhang);

    // ── Sub-feature 2: inner bore (smaller dia, deeper) ──────────────────
    const TopoDS_Shape innerBore = pr::cylinder(
        outerAx, rInner, innerBoreDepth + kOverhang);

    // Concentric overlapping — fuse first, single cut.
    const TopoDS_Shape stepFused = pr::fuse(outerBore, innerBore);

    // ── Sub-feature 3: O-ring groove on the shoulder wall ────────────────
    // The shoulder wall is the cylindrical surface of radius rOuter, from
    // z=topZ down to z=topZ-ledge_depth.  Groove = annular ring whose
    // radial extent crosses that wall outward (rOuter → rOuter+groove_depth).
    const double rGrooveOuter = rOuter + in.o_ring_depth_mm;
    const double rGrooveInner = rOuter - 0.02;   // bias to avoid coplanar
    const gp_Pnt grooveStart(in.center_x_mm, in.center_y_mm,
                             topZ - in.o_ring_z_offset_mm + kOverhang);
    const gp_Ax2 grooveAx(grooveStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape oRingGroove = pr::annularRing(
        grooveAx, rGrooveOuter, rGrooveInner, in.o_ring_cs_mm);

    // Fuse all three cutters then single cut.
    const TopoDS_Shape totalCutter = pr::fuse(stepFused, oRingGroove);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), totalCutter);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",         in.center_x_mm },
        { "center_y_mm",         in.center_y_mm },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "glass_dia_mm",        in.glass_dia_mm },
        { "ledge_depth_mm",      in.ledge_depth_mm },
        { "inner_bore_dia_mm",   innerBoreDia },
        { "inner_bore_depth_mm", innerBoreDepth },
        { "o_ring_cs_mm",        in.o_ring_cs_mm },
        { "o_ring_depth_mm",     in.o_ring_depth_mm },
        { "o_ring_z_offset_mm",  in.o_ring_z_offset_mm },
    };

    const double ledgeWidth = (in.glass_dia_mm - innerBoreDia) / 2.0;
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           3 },
        { "subfeatures", json::array({
            { { "op", "outer_bore_slipfit" }, { "dia_mm", 2.0 * rOuter },
              { "depth_mm", in.ledge_depth_mm } },
            { { "op", "inner_bore_ledge" },   { "dia_mm", innerBoreDia },
              { "depth_mm", innerBoreDepth } },
            { { "op", "shoulder_o_ring_groove" },
              { "cs_mm", in.o_ring_cs_mm },
              { "depth_mm", in.o_ring_depth_mm } },
        }) },
        { "glass_dia_mm",               in.glass_dia_mm },
        { "outer_bore_dia_mm",          2.0 * rOuter },
        { "inner_bore_dia_mm",          innerBoreDia },
        { "ledge_width_mm",             ledgeWidth },
        { "shoulder_height_mm",         in.ledge_depth_mm },
        { "cylindrical_face_count",     2 },          // outer + inner walls
        { "annular_face_count",         3 },          // ledge top + 2 groove walls
        { "axis_dir",                   { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "bore;groove_tool";
    tooling.tool_dia_mm       = 2.0 * rOuter;
    tooling.tool_length_mm    = innerBoreDepth + 3.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double outerVol = M_PI * rOuter * rOuter * in.ledge_depth_mm;
    const double innerVol = M_PI * rInner * rInner *
        (innerBoreDepth - in.ledge_depth_mm);
    const double grooveVol = M_PI *
        (rGrooveOuter * rGrooveOuter - rOuter * rOuter) * in.o_ring_cs_mm;
    tooling.stock_removed_mm3 = outerVol + innerVol + grooveVol;
    tooling.est_cycle_time_s  = std::max(3.0, tooling.stock_removed_mm3 / 80.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "bore" }, { "tool_dia_mm", 2.0 * rOuter },
              { "depth_mm", in.ledge_depth_mm } },
            { { "tool_type", "bore" }, { "tool_dia_mm", innerBoreDia },
              { "depth_mm", innerBoreDepth } },
            { { "tool_type", "groove_tool" }, { "width_mm", in.o_ring_cs_mm },
              { "depth_mm", in.o_ring_depth_mm } },
        } },
        { "feature_standard", "AS568 -004 family glass seat O-ring seal" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::sapphire_glass_seat applied: glass={} ledgeD={} innerD={}",
                  in.glass_dia_mm, in.ledge_depth_mm, innerBoreDia);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::sapphire_glass_seat
