// @lat: [[engine/skills#yaw_brake_caliper_mount]]

#include "yaw_brake_caliper_mount.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>
#include <memory>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::yaw_brake_caliper_mount {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pad_spacing_mm <= 0.0 || in.bolt_dia_mm <= 0.0 ||
        in.disc_slot_width_mm <= 0.0 || in.disc_slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "yaw_brake_caliper_mount: all dims must be > 0");
        return r;
    }

    const auto* spec = thread_table::findMetric(in.bolt_thread_key);
    if (!spec) {
        r.add("DFM-THREAD", "error",
              "yaw_brake_caliper_mount: bolt_thread_key '" +
              in.bolt_thread_key + "' not in _iso_thread_table");
    } else if (in.bolt_dia_mm < spec->nominal_dia_mm) {
        r.add("DFM-BOLT-FIT", "error",
              "yaw_brake_caliper_mount: bolt_dia " +
              std::to_string(in.bolt_dia_mm) +
              " mm < thread nominal " +
              std::to_string(spec->nominal_dia_mm) + " mm");
    }

    // Disc clearance slot must fit between the two pads.
    if (in.disc_slot_width_mm >= in.pad_spacing_mm) {
        r.add("DFM-SLOT", "error",
              "yaw_brake_caliper_mount: disc_slot_width " +
              std::to_string(in.disc_slot_width_mm) +
              " mm >= pad_spacing " + std::to_string(in.pad_spacing_mm) +
              " mm (slot would consume the bolt pads)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "yaw_brake_caliper_mount DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx   = in.center_xy.X();
    const double cy   = in.center_xy.Y();

    const double boltR    = in.bolt_dia_mm / 2.0;
    const double padHalf  = in.pad_spacing_mm / 2.0;
    const double rowOff   = in.pad_spacing_mm * 0.25;  // 2 holes per pad in Y
    const double boltDepth = std::min((zMax - zMin) * 0.8, 50.0);

    // ── 1..4) Four caliper bolt holes (2 pads × 2 holes) ─────────────────
    const std::array<std::pair<double, double>, 4> boltPositions {{
        { cx - padHalf, cy - rowOff },
        { cx - padHalf, cy + rowOff },
        { cx + padHalf, cy - rowOff },
        { cx + padHalf, cy + rowOff },
    }};

    TopoDS_Shape current = wp.shape();
    for (const auto& [bx, by] : boltPositions) {
        const gp_Pnt start(bx, by, topZ - boltDepth);
        const gp_Ax2 ax(start, gp::DZ());
        const TopoDS_Shape boltTool = pr::cylinder(ax, boltR, boltDepth + kOver);
        current = pr::cut(current, boltTool);   // sequential — no compound
    }

    // ── 5) Brake-disc clearance slot (box pocket) ────────────────────────
    // Slot runs along Y (disc passes through), width along X centred on cx.
    const double slotLenY = (yMax - yMin) + 2.0 * kOver;
    const gp_Pnt slotOrigin(
        cx - in.disc_slot_width_mm / 2.0,
        yMin - kOver,
        topZ - in.disc_slot_depth_mm);
    const TopoDS_Shape slotTool = pr::box(
        gp_Ax2(slotOrigin, gp::DZ()),
        in.disc_slot_width_mm, slotLenY, in.disc_slot_depth_mm + kOver);
    current = pr::cut(current, slotTool);

    // ── Derived volume (analytic) ────────────────────────────────────────
    const double vBolts = 4.0 * M_PI * boltR * boltR * boltDepth;
    const double vSlot  = in.disc_slot_width_mm * (yMax - yMin) *
                          in.disc_slot_depth_mm;
    const double volRemoved = vBolts + vSlot;

    json bolts_json = json::array();
    for (const auto& [bx, by] : boltPositions)
        bolts_json.push_back({ { "x_mm", bx }, { "y_mm", by } });

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "pad_spacing_mm",     in.pad_spacing_mm },
        { "bolt_thread_key",    in.bolt_thread_key },
        { "bolt_dia_mm",        in.bolt_dia_mm },
        { "disc_slot_width_mm", in.disc_slot_width_mm },
        { "disc_slot_depth_mm", in.disc_slot_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "wind_feature_type",          "yaw_brake_caliper_mount" },
        { "subfeature_count",           4 + 1 },
        { "bolt_count",                 4 },
        { "pad_spacing_mm",             in.pad_spacing_mm },
        { "bolt_thread_key",            in.bolt_thread_key },
        { "disc_slot_width_mm",         in.disc_slot_width_mm },
        { "bolt_positions",             bolts_json },
        { "derived_bolt_depth_mm",      boltDepth },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "IEC 61400-1 yaw system" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;slot_mill";
    tooling.tool_dia_mm       = in.bolt_dia_mm;
    tooling.tool_length_mm    = boltDepth + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 110.0;
    tooling.extra = {
        { "wind_feature_type", "yaw_brake_caliper_mount" },
        { "brake_family",      "hydraulic_caliper" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::yaw_brake_caliper_mount: pad_spacing={} thread={} slot={}",
                  in.pad_spacing_mm, in.bolt_thread_key, in.disc_slot_width_mm);

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
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "wind_feature_type", "yaw_brake_caliper_mount" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: 4 equal-radius +Z cylinders (bolt holes).
    int boltCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9) ++boltCyls;
        } catch (...) {}
    }
    if (boltCyls >= 4) {
        json recovered = { { "bolt_thread_key", "M16" } };
        json matched   = { { "source",    "geometric_4bolt_pattern" },
                           { "bolt_cyls", boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::yaw_brake_caliper_mount
