// @lat: [[engine/skills#pantograph_carbon_strip_groove]]

#include "pantograph_carbon_strip_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pantograph_carbon_strip_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// Clip retention slot is transversely wider than the groove (to form the
// undercut shoulders) and a bit deeper than the groove floor.
constexpr double kSlotWidthFrac = 1.40;   // × groove_width (across the bar)
constexpr double kSlotExtraDepth = 3.0;   // mm below groove floor
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.strip_length_mm <= 0.0 || in.groove_width_mm <= 0.0 ||
        in.groove_depth_mm <= 0.0 || in.clip_slot_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pantograph_carbon_strip_groove: all dimensions must be > 0");
    }

    if (in.clip_count < 1 || in.clip_count > 24) {
        r.add("DFM-COUNT", "error",
              "pantograph_carbon_strip_groove: clip_count (" +
              std::to_string(in.clip_count) + ") must be in [1, 24]");
    }

    if (in.clip_slot_width_mm >= in.groove_width_mm) {
        r.add("DFM-WIDTH", "error",
              "pantograph_carbon_strip_groove: clip_slot_width_mm must be "
              "smaller than groove_width_mm");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pantograph_carbon_strip_groove DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double ox = in.strip_origin.X();
    const double oy = in.strip_origin.Y();

    // ── 1) Carbon-strip groove (long box along +X) ────────────────────
    const double gLen   = in.strip_length_mm;
    const double gWid   = in.groove_width_mm;
    const double gDepth = in.groove_depth_mm;
    const gp_Pnt gOrigin(ox, oy - gWid / 2.0, topZ - gDepth);
    const gp_Ax2 gAx(gOrigin, gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(),
                                   pr::box(gAx, gLen, gWid, gDepth + kOver));

    // ── 2..N) Clip retention slots spaced along the groove ────────────
    // Each slot is a transverse box, wider than the groove (undercut), a
    // bit deeper than the groove floor.  Spaced evenly; cut SEQUENTIALLY.
    const double slotW   = in.clip_slot_width_mm;        // along X (groove dir)
    const double slotWid = gWid * kSlotWidthFrac;         // across bar (Y)
    const double slotDep = gDepth + kSlotExtraDepth;
    const double span    = gLen - slotW;
    for (int i = 0; i < in.clip_count; ++i) {
        const double frac = (in.clip_count == 1)
            ? 0.5
            : static_cast<double>(i) / static_cast<double>(in.clip_count - 1);
        const double slotCx = ox + slotW / 2.0 + frac * std::max(0.0, span);
        const gp_Pnt slotOrigin(slotCx - slotW / 2.0,
                                oy - slotWid / 2.0,
                                topZ - slotDep);
        const gp_Ax2 slotAx(slotOrigin, gp::DZ());
        current = pr::cut(current, pr::box(slotAx, slotW, slotWid, slotDep + kOver));
    }

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ─────────────────────────────────────
    const double vGroove = gLen * gWid * gDepth;
    // Each clip slot only adds the material outside the groove footprint
    // (extra width band + the extra depth band under the groove).
    const double vSlotExtra =
        slotW * (slotWid - gWid) * slotDep      // the wider transverse band
        + slotW * gWid * kSlotExtraDepth;        // the deeper band under groove
    const double vSlots = static_cast<double>(in.clip_count) *
                          std::max(0.0, vSlotExtra);
    const double volRemoved = vGroove + vSlots;

    const int subfeatureCount = 1 + in.clip_count;

    json clips_json = json::array();
    for (int i = 0; i < in.clip_count; ++i) {
        const double frac = (in.clip_count == 1)
            ? 0.5
            : static_cast<double>(i) / static_cast<double>(in.clip_count - 1);
        clips_json.push_back({
            { "x_mm",   ox + slotW / 2.0 + frac * std::max(0.0, span) },
            { "frac",   frac },
        });
    }

    json params = {
        { "strip_origin",         { in.strip_origin.X(),
                                    in.strip_origin.Y(),
                                    in.strip_origin.Z() } },
        { "strip_length_mm",      in.strip_length_mm },
        { "groove_width_mm",      in.groove_width_mm },
        { "groove_depth_mm",      in.groove_depth_mm },
        { "clip_count",           in.clip_count },
        { "clip_slot_width_mm",   in.clip_slot_width_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "railway_feature_type",       "pantograph_carbon_strip_groove" },
        { "subfeature_count",           subfeatureCount },
        { "strip_length_mm",            in.strip_length_mm },
        { "groove_width_mm",            in.groove_width_mm },
        { "groove_depth_mm",            in.groove_depth_mm },
        { "clip_count",                 in.clip_count },
        { "clip_positions",             clips_json },
        { "derived_slot_width_mm",      slotW },
        { "derived_slot_transverse_mm", slotWid },
        { "derived_slot_depth_mm",      slotDep },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "pantograph carbon-strip mounting groove" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "slot_mill;end_mill";
    tooling.tool_dia_mm       = std::min(in.groove_width_mm, in.clip_slot_width_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(40.0, in.strip_length_mm / 15.0)
                                + 5.0 * static_cast<double>(in.clip_count);
    tooling.extra = {
        { "railway_application", "pantograph_carbon_strip_groove" },
        { "clip_count",          in.clip_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pantograph_carbon_strip_groove: len={} width={} clips={} faces {}→{}",
                  in.strip_length_mm, in.groove_width_mm, in.clip_count,
                  wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: count planar faces near the top — a long groove
    // plus clip slots leaves many planar walls below the top face.
    int planarBelowTop = 0;
    double topZ = -1e30;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double pz = s.Plane().Location().Z();
            topZ = std::max(topZ, pz);
        } catch (...) {}
    }
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double pz = s.Plane().Location().Z();
            if (pz < topZ - 1.0) ++planarBelowTop;
        } catch (...) {}
    }
    if (planarBelowTop >= 4) {
        json recovered = { { "strip_length_mm",     1000.0 },
                           { "groove_width_mm",      30.0 },
                           { "groove_depth_mm",      12.0 },
                           { "clip_count",           6 },
                           { "clip_slot_width_mm",   6.0 } };
        json matched   = { { "source",            "geometric_pantograph_groove" },
                           { "planar_below_top",  planarBelowTop } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::pantograph_carbon_strip_groove
