// @lat: [[engine/skills#detented_position_slider]]

#include "detented_position_slider.hpp"

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
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::detented_position_slider {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec table (detent classes) ──────────────────────────────────────────

namespace {

constexpr std::array<DetentSpec, 4> kDetentTable {{
    { "FINE_3",    3, 1.5, 0.4 },
    { "STD_5",     5, 2.0, 0.6 },
    { "MED_7",     7, 2.5, 0.8 },
    { "COARSE_10", 10, 3.0, 1.0 },
}};

}  // namespace

const DetentSpec* lookupDetentSpec(const std::string& key)
{
    for (const auto& s : kDetentTable) {
        if (key == s.key) return &s;
    }
    return nullptr;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const DetentSpec* spec = lookupDetentSpec(in.detent_class);
    if (!spec) {
        r.add("DFM-SPEC", "error",
              "detented_position_slider: unknown detent_class '" +
              in.detent_class +
              "' (expected FINE_3|STD_5|MED_7|COARSE_10)");
        return r;
    }

    if (in.slot_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "detented_position_slider: slot_width " +
              std::to_string(in.slot_width_mm) + " mm < 0.8 mm");
    }
    if (in.slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "detented_position_slider: slot_depth_mm must be > 0");
    }
    if (spec->dimple_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "detented_position_slider: dimple_dia " +
              std::to_string(spec->dimple_dia_mm) + " mm < 0.8 mm");
    }

    const double slotLen = std::hypot(in.slot_end_x_mm - in.slot_start_x_mm,
                                      in.slot_end_y_mm - in.slot_start_y_mm);
    if (slotLen <= 1e-6) {
        r.add("DFM-INPUT", "error",
              "detented_position_slider: slot endpoints coincide");
    }

    // DFM-DIM-PITCH: pitch between consecutive dimples must be ≥ 2 × dimple_dia
    // so that dimples don't overlap.
    if (spec->detent_count > 1 && slotLen > 0.0) {
        const double pitch = slotLen / static_cast<double>(spec->detent_count - 1);
        if (pitch < 2.0 * spec->dimple_dia_mm) {
            r.add("DFM-DIM-PITCH", "error",
                  "detented_position_slider: pitch " + std::to_string(pitch) +
                  " mm < 2 × dimple_dia " +
                  std::to_string(2.0 * spec->dimple_dia_mm) +
                  " mm (dimples would overlap; use a coarser class or a longer slot)");
        }
    }

    // Slot must accommodate the dimple width.
    if (in.slot_width_mm < spec->dimple_dia_mm + 0.2) {
        r.add("DFM-INPUT", "warning",
              "detented_position_slider: slot_width " +
              std::to_string(in.slot_width_mm) +
              " mm tight to dimple_dia " +
              std::to_string(spec->dimple_dia_mm) + " mm");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "detented_position_slider DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const DetentSpec* spec = lookupDetentSpec(in.detent_class);
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("detented_position_slider: entry_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    const gp_Vec dir2D(in.slot_end_x_mm - in.slot_start_x_mm,
                       in.slot_end_y_mm - in.slot_start_y_mm, 0.0);
    const double slotLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / slotLen, dir2D.Y() / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    // SUBFEATURE 1: linear SLOT cut.
    const gp_Pnt slotMid(
        (in.slot_start_x_mm + in.slot_end_x_mm) / 2.0,
        (in.slot_start_y_mm + in.slot_end_y_mm) / 2.0,
        zMax);
    const gp_Pnt slotOrigin(
        slotMid.X() - slotLen / 2.0 * xLoc.X() - in.slot_width_mm / 2.0 * yLoc.X(),
        slotMid.Y() - slotLen / 2.0 * xLoc.Y() - in.slot_width_mm / 2.0 * yLoc.Y(),
        zMax - in.slot_depth_mm);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ(), xLoc);
    const TopoDS_Shape slotBox = pr::box(
        slotAx, slotLen, in.slot_width_mm, in.slot_depth_mm + kOverhang);

    // SUBFEATURES 2..N+1: N DIMPLE CUTS along the slot floor.
    // Each dimple is a small cylindrical cut extending dimple_depth_mm
    // BELOW the slot floor.
    std::vector<TopoDS_Shape> dimples;
    dimples.reserve(spec->detent_count);
    for (int k = 0; k < spec->detent_count; ++k) {
        const double t = (spec->detent_count == 1)
                       ? 0.5
                       : static_cast<double>(k) / static_cast<double>(spec->detent_count - 1);
        const double xC =
            in.slot_start_x_mm + t * (in.slot_end_x_mm - in.slot_start_x_mm);
        const double yC =
            in.slot_start_y_mm + t * (in.slot_end_y_mm - in.slot_start_y_mm);
        const gp_Pnt dimpleTop(xC, yC, zMax - in.slot_depth_mm + kOverhang);
        const gp_Ax2 dimpleAx(dimpleTop, gp_Dir(0.0, 0.0, -1.0), gp::DX());
        const TopoDS_Shape dimple = pr::cylinder(
            dimpleAx, spec->dimple_dia_mm / 2.0,
            spec->dimple_depth_mm + 2.0 * kOverhang);
        dimples.push_back(dimple);
    }

    // Boolean composition: fuse all dimples + slot, then cut.
    TopoDS_Shape cutter = slotBox;
    for (const auto& d : dimples) cutter = pr::fuse(cutter, d);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Build signature.
    json detentPositions = json::array();
    for (int k = 0; k < spec->detent_count; ++k) {
        const double t = (spec->detent_count == 1)
                       ? 0.5
                       : static_cast<double>(k) / static_cast<double>(spec->detent_count - 1);
        detentPositions.push_back({
            { "x", in.slot_start_x_mm + t * (in.slot_end_x_mm - in.slot_start_x_mm) },
            { "y", in.slot_start_y_mm + t * (in.slot_end_y_mm - in.slot_start_y_mm) },
            { "index", k },
        });
    }

    json params = {
        { "entry_face_id",   *entryId },
        { "slot_start_x_mm", in.slot_start_x_mm },
        { "slot_start_y_mm", in.slot_start_y_mm },
        { "slot_end_x_mm",   in.slot_end_x_mm },
        { "slot_end_y_mm",   in.slot_end_y_mm },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "slot_width_mm",   in.slot_width_mm },
        { "slot_depth_mm",   in.slot_depth_mm },
        { "detent_class",    in.detent_class },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     1 + spec->detent_count },
        { "detent_class",         in.detent_class },
        { "detent_count",         spec->detent_count },
        { "dimple_dia_mm",        spec->dimple_dia_mm },
        { "dimple_depth_mm",      spec->dimple_depth_mm },
        { "slot_width_mm",        in.slot_width_mm },
        { "slot_depth_mm",        in.slot_depth_mm },
        { "length_mm",            slotLen },
        { "detent_pitch_mm",
            (spec->detent_count > 1)
                ? slotLen / static_cast<double>(spec->detent_count - 1)
                : 0.0 },
        { "detent_positions",     detentPositions },
    };
    ToolingMeta tooling;
    tooling.tool_type = "end_mill;ball_mill";
    tooling.tool_dia_mm = std::min(in.slot_width_mm, spec->dimple_dia_mm);
    tooling.tool_length_mm = in.slot_depth_mm + spec->dimple_depth_mm + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 2;
    tooling.cutting_speed_sfm = 420.0;
    tooling.feed_per_tooth_mm = 0.025;
    const double rDimp = spec->dimple_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        slotLen * in.slot_width_mm * in.slot_depth_mm +
        spec->detent_count * M_PI * rDimp * rDimp * spec->dimple_depth_mm;
    tooling.est_cycle_time_s = std::max(5.0,
        slotLen * 0.2 + spec->detent_count * 0.8);
    tooling.extra = {
        { "compound_chain", {
            { { "step", "linear_slot" }, { "tool", "end_mill" } },
            { { "step", "detent_dimples" },
              { "count", spec->detent_count },
              { "tool", "ball_mill" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::detented_position_slider applied: class={} N={} len={}",
                  in.detent_class, spec->detent_count, slotLen);

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

    // Geometric heuristic: count cylindrical faces with axes parallel to Z.
    std::vector<std::pair<int, double>> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder cyl = s.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-2) continue;
        cyls.emplace_back(i, 2.0 * cyl.Radius());
    }
    if (cyls.size() < 2) return out;

    // Match to nearest spec by detent count.
    const int N = static_cast<int>(cyls.size());
    const DetentSpec* best = nullptr;
    int bestErr = 1 << 20;
    for (const auto& s : kDetentTable) {
        const int err = std::abs(s.detent_count - N);
        if (err < bestErr) { bestErr = err; best = &s; }
    }
    if (!best) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    json recovered = {
        { "slot_start_x_mm", xMin + 2.0 },
        { "slot_start_y_mm", (yMin + yMax) / 2.0 },
        { "slot_end_x_mm",   xMax - 2.0 },
        { "slot_end_y_mm",   (yMin + yMax) / 2.0 },
        { "axis_dir",        { 0.0, 0.0, -1.0 } },
        { "slot_width_mm",   std::max(0.8, best->dimple_dia_mm + 0.5) },
        { "slot_depth_mm",   std::max(0.5, (zMax - zMin) * 0.2) },
        { "detent_class",    std::string(best->key) },
    };
    json matched = {
        { "dimple_cyl_count",  N },
        { "spec_match_err_N",  bestErr },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.60, matched });
    return out;
}

}  // namespace koocadcam::skill::detented_position_slider
