// @lat: [[engine/skills#cable_carrier_anchor_bracket]]

#include "cable_carrier_anchor_bracket.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::cable_carrier_anchor_bracket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr double kOver = 0.1;

// Build one elongated slot tool centred at (sx, sy), oriented along X, by
// FUSING a central box with two end-cap cylinders into a single watertight
// tool.  The fused pieces overlap (the cylinders sit on the box ends), which
// is exactly why we FUSE them — never compound-cut overlapping cutters.
TopoDS_Shape makeSlotTool(double sx, double sy, double zBottom,
                          double slotLen, double slotWid, double height)
{
    const double r = slotWid / 2.0;
    const double boxLen = std::max(slotLen - slotWid, 0.0);  // rectangular core

    const gp_Pnt boxOrigin(sx - boxLen / 2.0, sy - slotWid / 2.0, zBottom);
    TopoDS_Shape tool = pr::box(gp_Ax2(boxOrigin, gp::DZ()),
                                boxLen, slotWid, height);

    const gp_Ax2 capAxA(gp_Pnt(sx + boxLen / 2.0, sy, zBottom), gp::DZ());
    const gp_Ax2 capAxB(gp_Pnt(sx - boxLen / 2.0, sy, zBottom), gp::DZ());
    tool = pr::fuse(tool, pr::cylinder(capAxA, r, height));
    tool = pr::fuse(tool, pr::cylinder(capAxB, r, height));
    return tool;
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.chain_width_mm <= 0.0 || in.bolt_slot_len_mm <= 0.0 ||
        in.bolt_slot_wid_mm <= 0.0 || in.bolt_slot_spacing_mm <= 0.0 ||
        in.pin_bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cable_carrier_anchor_bracket: all dimensions must be > 0");
    }

    if (in.bolt_slot_len_mm <= in.bolt_slot_wid_mm) {
        r.add("DFM-ROBOTICS-SLOT", "error",
              "cable_carrier_anchor_bracket: bolt_slot_len_mm must exceed "
              "bolt_slot_wid_mm (an elongated slot, not a round hole)");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cable_carrier_anchor_bracket DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;
    const double zBottom = zMin - kOver;

    // The two slots straddle the bracket centre along X; the pin bore sits on
    // the centre, offset in +Y so it does not interfere with the slots.
    const double halfSpacing = in.bolt_slot_spacing_mm / 2.0;
    const double pinOffsetY = in.chain_width_mm / 2.0 + in.bolt_slot_wid_mm;

    // ── 1) Bolt slot A (fused slot tool, then sequential cut) ──────────
    TopoDS_Shape current = wp.shape();
    {
        const TopoDS_Shape slotA = makeSlotTool(
            cx - halfSpacing, cy, zBottom,
            in.bolt_slot_len_mm, in.bolt_slot_wid_mm, thru);
        current = pr::cut(current, slotA);  // sequential
    }

    // ── 2) Bolt slot B (fused slot tool, then sequential cut) ──────────
    {
        const TopoDS_Shape slotB = makeSlotTool(
            cx + halfSpacing, cy, zBottom,
            in.bolt_slot_len_mm, in.bolt_slot_wid_mm, thru);
        current = pr::cut(current, slotB);  // sequential
    }

    // ── 3) Chain-link pin bore ─────────────────────────────────────────
    const double pinR = in.pin_bore_dia_mm / 2.0;
    const gp_Ax2 pinAx(gp_Pnt(cx, cy + pinOffsetY, zBottom), gp::DZ());
    current = pr::cut(current, pr::cylinder(pinAx, pinR, thru));  // sequential

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double plateThk = (zMax - zMin);
    const double slotArea = (in.bolt_slot_len_mm - in.bolt_slot_wid_mm) *
                                in.bolt_slot_wid_mm
                          + M_PI * std::pow(in.bolt_slot_wid_mm / 2.0, 2);
    const double vSlots = 2.0 * slotArea * plateThk;
    const double vPin   = M_PI * pinR * pinR * plateThk;
    const double volRemoved = vSlots + vPin;

    const int subfeatureCount = 2 + 1;

    json params = {
        { "center_xy",            { in.center_xy.X(),
                                    in.center_xy.Y(),
                                    in.center_xy.Z() } },
        { "chain_width_mm",       in.chain_width_mm },
        { "bolt_slot_len_mm",     in.bolt_slot_len_mm },
        { "bolt_slot_wid_mm",     in.bolt_slot_wid_mm },
        { "bolt_slot_spacing_mm", in.bolt_slot_spacing_mm },
        { "pin_bore_dia_mm",      in.pin_bore_dia_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "robotics_feature_type",      "energy_chain_end_anchor" },
        { "subfeature_count",           subfeatureCount },
        { "chain_width_mm",             in.chain_width_mm },
        { "bolt_slot_len_mm",           in.bolt_slot_len_mm },
        { "bolt_slot_wid_mm",           in.bolt_slot_wid_mm },
        { "pin_bore_dia_mm",            in.pin_bore_dia_mm },
        { "derived_slot_spacing_mm",    in.bolt_slot_spacing_mm },
        { "derived_pin_offset_y_mm",    pinOffsetY },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "energy chain end-anchor bracket practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::max(in.bolt_slot_wid_mm, in.pin_bore_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 210.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 55.0;
    tooling.extra = {
        { "robotics_application", "energy_chain_anchor" },
        { "removed_volume_mm3",   volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cable_carrier_anchor_bracket: chain_w={} slots=2 pin={} faces {}→{}",
                  in.chain_width_mm, in.pin_bore_dia_mm,
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

    // Geometric fallback: slot end-caps (small cyls) plus a single pin bore.
    int capCyls = 0;
    int pinCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 2.0 && radius < 4.0) ++capCyls;
            else if (radius >= 4.0 && radius < 8.0) ++pinCyls;
        } catch (...) {}
    }
    if (capCyls >= 2 && pinCyls >= 1) {
        json recovered = { { "bolt_slot_len_mm", 14.0 },
                           { "pin_bore_dia_mm",  8.0 } };
        json matched   = { { "source",   "geometric_anchor_bracket" },
                           { "cap_cyls", capCyls },
                           { "pin_cyls", pinCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::cable_carrier_anchor_bracket
