// @lat: [[engine/skills#rack_ear_19inch_eia]]

#include "rack_ear_19inch_eia.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
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

namespace koocadcam::skill::rack_ear_19inch_eia {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// EIA-310 universal 1U group spacing (mm): 0.500" / 0.625" / 0.500".
constexpr double kGap1 = 12.7;
constexpr double kGap2 = 15.875;

// Resolve the clearance diameter for an optional thread key.  Returns < 0 if
// the key is non-empty and unknown.
double clearanceDiaFor(const std::string& key, double fallback)
{
    if (key.empty()) return fallback;
    if (const tt::MetricThreadSpec* m = tt::findMetric(key))
        return m->clearance_medium_mm;
    if (const tt::UncUnfSpec* u = tt::findUncUnf(key))
        return u->clearance_normal_mm;
    return -1.0;
}
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.hole_dia_mm > 0.0) || !(in.handle_slot_len_mm > 0.0) ||
        !(in.handle_slot_wid_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "rack_ear_19inch_eia: hole_dia and handle slot dims must be > 0");
        return r;
    }
    if (!in.hole_thread_key.empty() &&
        clearanceDiaFor(in.hole_thread_key, in.hole_dia_mm) < 0.0) {
        r.add("DFM-THREAD", "error",
              "rack_ear_19inch_eia: hole_thread_key '" + in.hole_thread_key +
              "' not in central metric or UNC/UNF table");
    }
    if (!(in.handle_slot_len_mm > in.handle_slot_wid_mm)) {
        r.add("DFM-SLOT", "error",
              "rack_ear_19inch_eia: handle_slot_len must exceed handle_slot_wid");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rack_ear_19inch_eia DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;

    const double clearanceDia = clearanceDiaFor(in.hole_thread_key, in.hole_dia_mm);
    const double holeR = clearanceDia / 2.0;

    // ── 1..3) Three EIA mounting holes along +Y ──────────────────────────
    // Group span = kGap1 + kGap2; center the group on cy.
    const double span = kGap1 + kGap2;
    const double y0 = cy - span / 2.0;
    const double yHoles[3] = { y0, y0 + kGap1, y0 + kGap1 + kGap2 };

    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < 3; ++i) {
        const gp_Pnt holeStart(cx, yHoles[i], zMin - kOver);
        const gp_Ax2 holeAx(holeStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(holeAx, holeR, thru));
    }

    // ── 4) Handle slot (rounded-rect pocket) below the hole group ────────
    const double slotDepth = std::min((zMax - zMin) * 0.5, 6.0) + kOver;
    const double slotY = cy + span / 2.0 + in.handle_slot_wid_mm;
    const gp_Pnt slotBottom(cx, slotY, topZ - slotDepth);
    const double cornerR = in.handle_slot_wid_mm * 0.45;   // < width/2 for safe fillet
    const TopoDS_Shape slotTool = pr::roundedRectPocketTool(
        slotBottom, in.handle_slot_len_mm, in.handle_slot_wid_mm,
        slotDepth, cornerR);
    current = pr::cut(current, slotTool);

    const int subfeatures = 3 + 1;
    const double vHoles = M_PI * holeR * holeR * (zMax - zMin) * 3.0;
    const double vSlot  = in.handle_slot_len_mm * in.handle_slot_wid_mm *
                          (slotDepth - kOver);
    const double volRemoved = vHoles + vSlot;

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "hole_dia_mm",        in.hole_dia_mm },
        { "hole_thread_key",    in.hole_thread_key },
        { "handle_slot_len_mm", in.handle_slot_len_mm },
        { "handle_slot_wid_mm", in.handle_slot_wid_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "audio_feature_type",         "rack_ear_eia310" },
        { "subfeature_count",           subfeatures },
        { "derived_clearance_dia_mm",   clearanceDia },
        { "derived_eia_gap1_mm",        kGap1 },
        { "derived_eia_gap2_mm",        kGap2 },
        { "derived_handle_slot_len_mm", in.handle_slot_len_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "EIA-310" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;slot_mill";
    tooling.tool_dia_mm       = clearanceDia;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 260.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(15.0, 8.0 + in.handle_slot_len_mm / 5.0);
    tooling.extra = {
        { "audio_application", "rack_mount_ear" },
        { "standard",          "EIA-310" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rack_ear_19inch_eia: hole_dia={} thread='{}' slot={}",
                  in.hole_dia_mm, in.hole_thread_key, in.handle_slot_len_mm);

    return SkillOutput{ wpNew, sig };
}

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
    if (!out.empty()) return out;

    // Geometric: three small through holes (mounting) leave several small
    // cylindrical faces.
    int smallCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 2.0 && radius <= 6.0) ++smallCyls;
        } catch (...) {}
    }
    if (smallCyls >= 3) {
        json recovered = { { "hole_dia_mm",        6.6 },
                           { "hole_thread_key",    "M6" },
                           { "handle_slot_len_mm", 40.0 },
                           { "handle_slot_wid_mm", 10.0 } };
        json matched   = { { "source",     "geometric_eia_hole_group" },
                           { "small_cyls", smallCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::rack_ear_19inch_eia
