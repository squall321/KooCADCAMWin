// @lat: [[engine/skills#speaker_cone_mount_4hole]]

#include "speaker_cone_mount_4hole.hpp"

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

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::speaker_cone_mount_4hole {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

bool isStandardConeDia(double cone_dia_mm)
{
    // 4"/5"/6.5"/8"/10" speakers — typical OD mounting diameter.
    static constexpr double kStandard[] = { 100.0, 130.0, 165.0, 200.0, 250.0 };
    for (double v : kStandard) if (std::abs(cone_dia_mm - v) < 0.5) return true;
    return false;
}

// Lookup clearance dia from either metric (M-prefix) or UNC/UNF table.
static double clearanceDiaFor(const std::string& key)
{
    if (key.empty()) return 0.0;
    if (key.size() > 0 && (key[0] == 'M' || key[0] == 'm')) {
        const auto* s = tt::findMetric(key);
        return s ? s->clearance_medium_mm : 0.0;
    }
    const auto* u = tt::findUncUnf(key);
    return u ? u->clearance_normal_mm : 0.0;
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.cone_dia_mm <= 0.0 || in.mount_pcd_mm <= 0.0
        || in.panel_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "speaker_cone_mount_4hole: cone_dia, pcd, panel_thickness must be > 0");
    }
    if (!isStandardConeDia(in.cone_dia_mm)) {
        r.add("DFM-CONE-DIA", "error",
              "speaker_cone_mount_4hole: cone_dia "
              + std::to_string(in.cone_dia_mm)
              + " mm not in standard set {100, 130, 165, 200, 250}");
    }
    if (in.mount_bolt_count != 4 && in.mount_bolt_count != 8) {
        r.add("DFM-BOLT-COUNT", "error",
              "speaker_cone_mount_4hole: mount_bolt_count "
              + std::to_string(in.mount_bolt_count) + " not in {4, 8}");
    }
    if (in.bolt_thread_size_key.empty()
        || clearanceDiaFor(in.bolt_thread_size_key) <= 0.0) {
        r.add("DFM-THREAD-KEY", "error",
              "speaker_cone_mount_4hole: bolt_thread_size_key '"
              + in.bolt_thread_size_key
              + "' not found in central metric or UNC/UNF table");
        (void)wp;
        return r;
    }
    const double clrDia = clearanceDiaFor(in.bolt_thread_size_key);
    if (in.mount_pcd_mm <= in.cone_dia_mm + 2.0 * clrDia) {
        r.add("DFM-PCD", "error",
              "speaker_cone_mount_4hole: pcd must exceed cone_dia + 2 × bolt clearance");
    }
    if (in.recess_dia_mm < 0.0 || in.recess_depth_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "speaker_cone_mount_4hole: recess dims must be >= 0");
    }
    if (in.recess_dia_mm > 0.0 && in.recess_dia_mm <= in.cone_dia_mm) {
        r.add("DFM-RECESS", "error",
              "speaker_cone_mount_4hole: recess_dia must exceed cone_dia "
              "(rear bumper is wider than cone)");
    }
    if (!(std::abs(in.axis_dir.X()) < 1e-6 && std::abs(in.axis_dir.Y()) < 1e-6)) {
        r.add("DFM-AXIS", "error",
              "speaker_cone_mount_4hole: only Z-aligned axis_dir is supported");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "speaker_cone_mount_4hole DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double clrDia = clearanceDiaFor(in.bolt_thread_size_key);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kOver = 0.1;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;

    const bool fromTop = in.axis_dir.Z() < 0.0;
    const double entryZ = fromTop ? zMax : zMin;
    const gp_Dir axisDir = in.axis_dir;

    // Effective through-thickness for the cone cut (full panel + overhang).
    const double thickness = std::max(in.panel_thickness_mm, zMax - zMin);

    // ── 1) Central cone opening (through hole) ──────────────────────────
    const double coneR = in.cone_dia_mm / 2.0;
    const gp_Pnt coneOrigin(cx, cy, entryZ + (fromTop ? kOver : -kOver));
    const gp_Ax2 coneAx(coneOrigin, axisDir);
    const TopoDS_Shape coneTool = pr::cylinder(coneAx, coneR,
        thickness + 2.0 * kOver);
    TopoDS_Shape current = pr::cut(wp.shape(), coneTool);

    // ── 2..) N bolt clearance holes on PCD ──────────────────────────────
    const double boltR = clrDia / 2.0;
    const double pcdR  = in.mount_pcd_mm / 2.0;
    for (int i = 0; i < in.mount_bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i))
                           / static_cast<double>(in.mount_bolt_count);
        const double bx = cx + pcdR * std::cos(theta);
        const double by = cy + pcdR * std::sin(theta);
        const gp_Pnt boltOrigin(bx, by, entryZ + (fromTop ? kOver : -kOver));
        const gp_Ax2 boltAx(boltOrigin, axisDir);
        const TopoDS_Shape boltTool = pr::cylinder(boltAx, boltR,
            thickness + 2.0 * kOver);
        current = pr::cut(current, boltTool);
    }

    // ── 3) Optional rear bumper recess ─────────────────────────────────
    bool hasRecess = false;
    if (in.recess_dia_mm > 0.0 && in.recess_depth_mm > 0.0) {
        const double recessR = in.recess_dia_mm / 2.0;
        const gp_Pnt recessOrigin(cx, cy, entryZ + (fromTop ? kOver : -kOver));
        const gp_Ax2 recessAx(recessOrigin, axisDir);
        const TopoDS_Shape recessTool = pr::cylinder(recessAx, recessR,
            in.recess_depth_mm + 2.0 * kOver);
        current = pr::cut(current, recessTool);
        hasRecess = true;
    }

    // Volume estimate.
    const double vCone   = M_PI * coneR * coneR * thickness;
    const double vBolts  = in.mount_bolt_count
                         * M_PI * boltR * boltR * thickness;
    const double vRecess = hasRecess
        ? M_PI * (in.recess_dia_mm / 2.0) * (in.recess_dia_mm / 2.0)
          * in.recess_depth_mm
        : 0.0;
    const double volRemoved = vCone + vBolts + vRecess;

    // Bolt positions JSON.
    json bolts_json = json::array();
    for (int i = 0; i < in.mount_bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i))
                           / static_cast<double>(in.mount_bolt_count);
        bolts_json.push_back({
            { "x", cx + pcdR * std::cos(theta) },
            { "y", cy + pcdR * std::sin(theta) },
            { "dia_mm", clrDia },
        });
    }

    json params = {
        { "center_x_mm",          cx },
        { "center_y_mm",          cy },
        { "axis_dir",             { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "cone_dia_mm",          in.cone_dia_mm },
        { "mount_bolt_count",     in.mount_bolt_count },
        { "mount_pcd_mm",         in.mount_pcd_mm },
        { "bolt_thread_size_key", in.bolt_thread_size_key },
        { "recess_dia_mm",        in.recess_dia_mm },
        { "recess_depth_mm",      in.recess_depth_mm },
        { "panel_thickness_mm",   in.panel_thickness_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "audio_feature_type",  "speaker_cone_mount" },
        { "subfeature_count",    1 + in.mount_bolt_count + (hasRecess ? 1 : 0) },
        { "cone_dia_mm",         in.cone_dia_mm },
        { "mount_bolt_count",    in.mount_bolt_count },
        { "mount_pcd_mm",        in.mount_pcd_mm },
        { "bolt_thread_size_key", in.bolt_thread_size_key },
        { "bolt_clearance_dia_mm", clrDia },
        { "has_rear_bumper_recess", hasRecess },
        { "bolt_positions",      bolts_json },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "jigsaw;drill;counterbore";
    tooling.tool_dia_mm       = std::max(in.cone_dia_mm, clrDia);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(30.0,
        20.0 + 6.0 * static_cast<double>(in.mount_bolt_count));
    tooling.extra = {
        { "standard",           "speaker driver mounting pattern (4-hole / 8-hole)" },
        { "speaker_size_class", in.cone_dia_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::speaker_cone_mount_4hole: cone_dia={} bolts={} pcd={}",
                  in.cone_dia_mm, in.mount_bolt_count, in.mount_pcd_mm);

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

    // Geometric: find a large central axial cylinder + ≥ 4 small axial
    // cylinders on a PCD around it.
    struct CylRec { int id; double r; double cx; double cy; };
    std::vector<CylRec> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) < 0.9) continue;
            const auto loc = s.Cylinder().Axis().Location();
            cyls.push_back({ i, s.Cylinder().Radius(), loc.X(), loc.Y() });
        } catch (...) {}
    }
    if (cyls.size() < 5) return out;

    // Find the largest-radius cylinder = "cone opening" candidate.
    auto coneIt = std::max_element(cyls.begin(), cyls.end(),
        [](const CylRec& a, const CylRec& b) { return a.r < b.r; });
    const CylRec cone = *coneIt;

    int boltCount = 0;
    double sumPcdR = 0.0;
    for (const auto& c : cyls) {
        if (c.id == cone.id) continue;
        if (c.r >= cone.r) continue;
        const double dx = c.cx - cone.cx;
        const double dy = c.cy - cone.cy;
        const double pr = std::hypot(dx, dy);
        if (pr < 5.0) continue;
        ++boltCount;
        sumPcdR += pr;
    }
    if (boltCount >= 4) {
        const double pcdRavg = sumPcdR / static_cast<double>(boltCount);
        json recovered = {
            { "cone_dia_mm",      2.0 * cone.r },
            { "mount_bolt_count", boltCount },
            { "mount_pcd_mm",     2.0 * pcdRavg },
        };
        json matched = { { "source", "geometric_speaker_pcd_pattern" },
                         { "cone_face_id", cone.id },
                         { "bolt_count_detected", boltCount } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::speaker_cone_mount_4hole
