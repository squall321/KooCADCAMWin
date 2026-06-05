// @lat: [[engine/skills#rail_clip_pandrol_seat]]

#include "rail_clip_pandrol_seat.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::rail_clip_pandrol_seat {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// Clip housing pocket is a fraction of the shoulder recess footprint.
constexpr double kClipLenFrac   = 0.45;   // along X (rail running direction)
constexpr double kClipWidFrac   = 0.30;   // across the foot
constexpr double kClipDepthFrac = 1.40;   // deeper than the shoulder recess
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.rail_foot_width_mm <= 0.0 || in.clip_seat_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "rail_clip_pandrol_seat: all dimensions must be > 0");
    }

    if (in.bolt_thread_key.empty()) {
        r.add("DFM-THREAD", "error",
              "rail_clip_pandrol_seat: bolt_thread_key is empty");
    } else if (!tt::findMetric(in.bolt_thread_key)) {
        r.add("DFM-THREAD", "error",
              "rail_clip_pandrol_seat: bolt_thread_key '" + in.bolt_thread_key +
              "' not present in central _iso_thread_table.hpp");
    }

    if (in.clip_seat_depth_mm >= in.rail_foot_width_mm) {
        r.add("DFM-FOOT", "error",
              "rail_clip_pandrol_seat: clip_seat_depth_mm must be smaller than "
              "rail_foot_width_mm");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rail_clip_pandrol_seat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* mSpec = tt::findMetric(in.bolt_thread_key);
    if (!mSpec) throw SkillError("rail_clip_pandrol_seat: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Shoulder recess (rectangular box, locates the rail foot) ────
    const double recLen   = in.rail_foot_width_mm * 0.7;   // along X
    const double recWid   = in.rail_foot_width_mm;          // across foot (Y)
    const double recDepth = in.clip_seat_depth_mm;
    const gp_Pnt recOrigin(cx - recLen / 2.0,
                           cy - recWid / 2.0,
                           topZ - recDepth);
    const gp_Ax2 recAx(recOrigin, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::box(recAx, recLen, recWid, recDepth + kOver));

    // ── 2) Clip housing pocket (deeper, narrower, +X side of recess) ──
    const double clipLen   = recLen * kClipLenFrac;
    const double clipWid   = recWid * kClipWidFrac;
    const double clipDepth = recDepth * kClipDepthFrac;
    const double clipCx    = cx + recLen / 2.0 - clipLen / 2.0;
    const gp_Pnt clipOrigin(clipCx - clipLen / 2.0,
                            cy - clipWid / 2.0,
                            topZ - clipDepth);
    const gp_Ax2 clipAx(clipOrigin, gp::DZ());
    current = pr::cut(current, pr::box(clipAx, clipLen, clipWid, clipDepth + kOver));

    // ── 3) Anchor bolt clearance hole (M-thread, non-overlapping) ─────
    // Placed on the -X side, clear of both pockets, through the plate.
    const double boltClr = mSpec->clearance_medium_mm;
    const double boltR   = boltClr / 2.0;
    const double boltCx  = cx - recLen / 2.0 - boltR - 4.0;
    const double thru    = (zMax - zMin) + 2.0 * kOver;
    const gp_Ax2 boltAx(gp_Pnt(boltCx, cy, zMin - kOver), gp::DZ());
    current = pr::cut(current, pr::cylinder(boltAx, boltR, thru));

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ─────────────────────────────────────
    const double plateThk = (zMax - zMin);
    const double vRecess  = recLen * recWid * recDepth;
    const double vClip    = clipLen * clipWid * clipDepth;
    const double vBolt    = M_PI * boltR * boltR * plateThk;
    const double volRemoved = vRecess + vClip + vBolt;

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "rail_foot_width_mm", in.rail_foot_width_mm },
        { "clip_seat_depth_mm", in.clip_seat_depth_mm },
        { "bolt_thread_key",    in.bolt_thread_key },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "railway_feature_type",       "pandrol_eclip_rail_seat" },
        { "subfeature_count",           3 },
        { "rail_foot_width_mm",         in.rail_foot_width_mm },
        { "clip_seat_depth_mm",         in.clip_seat_depth_mm },
        { "bolt_thread_key",            in.bolt_thread_key },
        { "derived_recess_len_mm",      recLen },
        { "derived_recess_wid_mm",      recWid },
        { "derived_clip_len_mm",        clipLen },
        { "derived_clip_wid_mm",        clipWid },
        { "derived_clip_depth_mm",      clipDepth },
        { "derived_bolt_clearance_mm",  boltClr },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "Pandrol e-clip rail fastening" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::max(boltClr, clipWid);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(45.0, 60.0);
    tooling.extra = {
        { "railway_application", "pandrol_eclip_rail_seat" },
        { "bolt_thread_key",     in.bolt_thread_key },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rail_clip_pandrol_seat: foot={} depth={} thread={} faces {}→{}",
                  in.rail_foot_width_mm, in.clip_seat_depth_mm,
                  in.bolt_thread_key, wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: a Z-axis bolt clearance cylinder plus pocket
    // walls (planar faces) on the top — bolt + at least one small cyl.
    int boltCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 2.0 && radius <= 16.0) ++boltCyls;
        } catch (...) {}
    }
    if (boltCyls >= 1) {
        json recovered = { { "rail_foot_width_mm", 76.0 },
                           { "clip_seat_depth_mm", 10.0 },
                           { "bolt_thread_key",    "M20" } };
        json matched   = { { "source",    "geometric_pandrol_seat" },
                           { "bolt_cyls", boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::rail_clip_pandrol_seat
