// @lat: [[engine/skills#microinverter_mount_slots]]

#include "microinverter_mount_slots.hpp"

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

namespace koocadcam::skill::microinverter_mount_slots {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// Cut one obround slot (box + 2 end cylinders), oriented along X, centred at
// (cx, cy), through `thick` deep from `topZ` downward.  Returns new shape.
TopoDS_Shape cutObroundSlot(TopoDS_Shape current,
                            double cx, double cy, double topZ,
                            double slotLen, double slotWid, double thick)
{
    const double r = slotWid / 2.0;
    const double boxLen = std::max(0.0, slotLen - slotWid);  // box between caps

    // box body
    const gp_Pnt boxOrigin(cx - boxLen / 2.0, cy - r, topZ - thick - kOver);
    const gp_Ax2 boxAx(boxOrigin, gp::DZ());
    current = pr::cut(current, pr::box(boxAx, boxLen, slotWid, thick + 2.0 * kOver));

    // end cap 1 (−X)
    const gp_Pnt c1(cx - boxLen / 2.0, cy, topZ - thick - kOver);
    const gp_Ax2 a1(c1, gp::DZ());
    current = pr::cut(current, pr::cylinder(a1, r, thick + 2.0 * kOver));

    // end cap 2 (+X)
    const gp_Pnt c2(cx + boxLen / 2.0, cy, topZ - thick - kOver);
    const gp_Ax2 a2(c2, gp::DZ());
    current = pr::cut(current, pr::cylinder(a2, r, thick + 2.0 * kOver));

    return current;
}
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.slot_length_mm <= 0.0 || in.slot_width_mm <= 0.0 ||
        in.slot_spacing_mm <= 0.0 || in.notch_width_mm <= 0.0 ||
        in.notch_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "microinverter_mount_slots: all dimensions must be > 0");
    }
    if (in.slot_length_mm > 0.0 && in.slot_width_mm > 0.0 &&
        in.slot_length_mm <= in.slot_width_mm) {
        r.add("DFM-SLOTGEOM", "error",
              "microinverter_mount_slots: slot_length_mm " +
              std::to_string(in.slot_length_mm) +
              " must be > slot_width_mm " + std::to_string(in.slot_width_mm) +
              " (obround degenerates to a hole)");
    }
    if (in.slot_spacing_mm > 0.0 && in.slot_width_mm > 0.0 &&
        in.slot_spacing_mm <= in.slot_width_mm) {
        r.add("DFM-SPACING", "error",
              "microinverter_mount_slots: slot_spacing_mm " +
              std::to_string(in.slot_spacing_mm) +
              " must be > slot_width_mm " + std::to_string(in.slot_width_mm) +
              " (slots would overlap)");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "microinverter_mount_slots DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double thick = zMax - zMin;  // through slots
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── slots 0 and 1: obround, offset ±spacing/2 in Y ──────────────────
    TopoDS_Shape current = wp.shape();
    current = cutObroundSlot(current, cx, cy - in.slot_spacing_mm / 2.0,
                             topZ, in.slot_length_mm, in.slot_width_mm, thick);
    current = cutObroundSlot(current, cx, cy + in.slot_spacing_mm / 2.0,
                             topZ, in.slot_length_mm, in.slot_width_mm, thick);

    // ── notch: cable strain-relief box at +X edge of the slot cluster ───
    const double nW = in.notch_width_mm;
    const double nD = in.notch_depth_mm;
    const double notchLen = std::min(nD, (zMax - zMin));
    const double nx = cx + in.slot_length_mm / 2.0 + 2.0;
    const gp_Pnt nOrigin(nx, cy - nW / 2.0, topZ - notchLen);
    const gp_Ax2 nAx(nOrigin, gp::DZ());
    current = pr::cut(current, pr::box(nAx, nD, nW, notchLen + kOver));

    // Volume estimate.
    const double slotArea =
        std::max(0.0, in.slot_length_mm - in.slot_width_mm) * in.slot_width_mm +
        M_PI * (in.slot_width_mm / 2.0) * (in.slot_width_mm / 2.0);
    const double vSlots = 2.0 * slotArea * thick;
    const double vNotch = nD * nW * notchLen;
    const double volRemoved = vSlots + vNotch;

    json params = {
        { "center_xy",      { in.center_xy.X(), in.center_xy.Y(), in.center_xy.Z() } },
        { "slot_length_mm", in.slot_length_mm },
        { "slot_width_mm",  in.slot_width_mm },
        { "slot_spacing_mm", in.slot_spacing_mm },
        { "notch_width_mm", in.notch_width_mm },
        { "notch_depth_mm", in.notch_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "solar_feature_type",         "microinverter_mount_slots" },
        { "subfeature_count",           7 },
        { "slot_count",                 2 },
        { "derived_slot_length_mm",     in.slot_length_mm },
        { "derived_slot_width_mm",      in.slot_width_mm },
        { "derived_slot_spacing_mm",    in.slot_spacing_mm },
        { "derived_notch_width_mm",     in.notch_width_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "UL 1741" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "slot_mill;drill";
    tooling.tool_dia_mm       = in.slot_width_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(25.0, in.slot_length_mm + 10.0);
    tooling.extra = {
        { "solar_application", "microinverter_bracket" },
        { "standard",          "UL 1741" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::microinverter_mount_slots: slot {}x{} spacing={}",
                  in.slot_length_mm, in.slot_width_mm, in.slot_spacing_mm);

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

    // Geometric: obround slots leave at least 4 vertical end-cap cylinders.
    int endCapCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 2.0 && radius <= 8.0) ++endCapCyls;
        } catch (...) {}
    }
    if (endCapCyls >= 3) {
        json recovered = { { "slot_length_mm", 20.0 },
                           { "slot_width_mm",  7.0 },
                           { "slot_spacing_mm", 30.0 },
                           { "notch_width_mm", 8.0 },
                           { "notch_depth_mm", 6.0 } };
        json matched = { { "source", "geometric_obround_slots" },
                         { "end_cap_cyls", endCapCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::microinverter_mount_slots
