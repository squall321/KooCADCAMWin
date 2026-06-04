// @lat: [[engine/skills#manifold_block_compound]]

#include "manifold_block_compound.hpp"

#include "Workpiece.hpp"
#include "_hydraulic_ports.hpp"
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

namespace koocadcam::skill::manifold_block_compound {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hyd_ports;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// Choose any axis perpendicular to `mainDir`.  For mainDir=+X we use +Z;
// for any orthonormal mainDir we cross with the global axis that is least
// parallel and renormalize.
gp_Dir perpAxis(const gp_Dir& mainDir)
{
    if (std::abs(mainDir.X()) > 0.9) return gp::DZ();
    if (std::abs(mainDir.Y()) > 0.9) return gp::DZ();
    return gp::DY();
}
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.main_dia_mm <= 0.0 || in.main_length_mm <= 0.0
        || in.branch_dia_mm <= 0.0 || in.branch_pitch_mm <= 0.0
        || in.branch_count < 1) {
        r.add("DFM-INPUT", "error",
              "manifold_block_compound: positive dims + branch_count >= 1 required");
    }
    if (in.branch_thread_size_key.empty()) {
        r.add("DFM-BSP-CODE", "error",
              "manifold_block_compound: branch_thread_size_key is empty");
    } else if (!ht::findBspG(in.branch_thread_size_key)) {
        r.add("DFM-BSP-CODE", "error",
              "manifold_block_compound: unknown BSP G size '" +
              in.branch_thread_size_key + "' (must be in central BSP G table)");
    }
    if (in.main_length_mm > 0.0 && in.branch_count > 1
        && (in.branch_count - 1) * in.branch_pitch_mm > in.main_length_mm) {
        r.add("DFM-BRANCH-FIT", "error",
              "manifold_block_compound: branch_count × pitch exceeds main_length");
    }
    if (in.branch_dia_mm >= in.main_dia_mm) {
        r.add("DFM-BRANCH-DIA", "error",
              "manifold_block_compound: branch_dia must be less than main_dia");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "manifold_block_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::BspGSpec* bs = ht::findBspG(in.branch_thread_size_key);
    if (!bs) throw SkillError("manifold_block_compound: BSP lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir mainDir = in.main_axis_dir;
    const gp_Dir branchDir = perpAxis(mainDir);
    const double mainR = in.main_dia_mm / 2.0;

    // ── 1) main bore ──────────────────────────────────────────────────────
    const gp_Pnt mainStart(
        in.main_bore_origin.X() - mainDir.X() * kOver,
        in.main_bore_origin.Y() - mainDir.Y() * kOver,
        in.main_bore_origin.Z() - mainDir.Z() * kOver);
    const gp_Ax2 mainAx(mainStart, mainDir);
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(mainAx, mainR, in.main_length_mm + 2.0 * kOver));

    // ── 2..) N branch ports ───────────────────────────────────────────────
    //
    // Each branch is positioned along the main axis at i * branch_pitch from
    // the main origin.  Branch axis = branchDir (perpendicular).  Branch
    // drill goes from outside the block INWARD to the main bore axis (depth
    // = enough to reach center + a safety overhang).
    //
    // Branch drill radius = max(branch_dia/2, BSP tap_drill/2).
    const double drillR = std::max(in.branch_dia_mm / 2.0,
                                   bs->drill_dia_mm / 2.0);
    const double spotR  = bs->spotface_dia_mm / 2.0;
    const double bDepth = bs->depth_mm;

    // Distance from main bore axis to the far face of the block along
    // branchDir (used to ensure the drill cuts cleanly through).
    auto faceDist = [&](const gp_Pnt& p, const gp_Dir& d) {
        if (std::abs(std::abs(d.X()) - 1.0) < 1e-6)
            return (d.X() > 0 ? xMax - p.X() : p.X() - xMin);
        if (std::abs(std::abs(d.Y()) - 1.0) < 1e-6)
            return (d.Y() > 0 ? yMax - p.Y() : p.Y() - yMin);
        if (std::abs(std::abs(d.Z()) - 1.0) < 1e-6)
            return (d.Z() > 0 ? zMax - p.Z() : p.Z() - zMin);
        return std::max(xMax - xMin, std::max(yMax - yMin, zMax - zMin));
    };

    for (int i = 0; i < in.branch_count; ++i) {
        // Branch centre on main axis
        const gp_Pnt centre(
            in.main_bore_origin.X() + mainDir.X() * i * in.branch_pitch_mm,
            in.main_bore_origin.Y() + mainDir.Y() * i * in.branch_pitch_mm,
            in.main_bore_origin.Z() + mainDir.Z() * i * in.branch_pitch_mm);

        const double fd = faceDist(centre, branchDir);
        const double drillLen = fd + 2.0 * kOver;
        const gp_Pnt drillStart(
            centre.X() + branchDir.X() * (fd - drillLen + kOver),
            centre.Y() + branchDir.Y() * (fd - drillLen + kOver),
            centre.Z() + branchDir.Z() * (fd - drillLen + kOver));
        const gp_Ax2 drillAx(drillStart, branchDir);
        const TopoDS_Shape drill = pr::cylinder(drillAx, drillR, drillLen);

        // Spotface dish (BSP G uses a spotface for the bonded seal washer).
        // Anchored at the outside face going INWARD bDepth.
        const gp_Pnt spotStart(
            centre.X() + branchDir.X() * (fd - bDepth),
            centre.Y() + branchDir.Y() * (fd - bDepth),
            centre.Z() + branchDir.Z() * (fd - bDepth));
        const gp_Ax2 spotAx(spotStart, branchDir);
        const TopoDS_Shape spotFace = pr::cylinder(spotAx, spotR, bDepth + kOver);

        // Pre-fuse drill + spotface (concentric overlap) → single cut.
        // Matches manifold_cross_drill_compound pattern (slice-14 fix).
        const TopoDS_Shape unified = pr::fuse(drill, spotFace);
        current = pr::cut(current, unified);
    }

    const double vMain = M_PI * mainR * mainR * in.main_length_mm;
    const double avgFD = std::max(xMax - xMin, std::max(yMax - yMin, zMax - zMin));
    const double vBranch = in.branch_count
                           * (M_PI * drillR * drillR * (avgFD * 0.5)
                              + M_PI * (spotR * spotR - drillR * drillR) * bDepth);
    const double volRemoved = vMain + vBranch;

    json params = {
        { "main_bore_origin",       { in.main_bore_origin.X(),
                                      in.main_bore_origin.Y(),
                                      in.main_bore_origin.Z() } },
        { "main_axis_dir",          { mainDir.X(), mainDir.Y(), mainDir.Z() } },
        { "main_dia_mm",            in.main_dia_mm },
        { "main_length_mm",         in.main_length_mm },
        { "branch_count",           in.branch_count },
        { "branch_dia_mm",          in.branch_dia_mm },
        { "branch_pitch_mm",        in.branch_pitch_mm },
        { "branch_thread_size_key", in.branch_thread_size_key },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "is_compound",              true },
        { "hvac_feature_type",        "hydraulic_manifold_block" },
        { "subfeature_count",         1 + in.branch_count },
        { "main_dia_mm",              in.main_dia_mm },
        { "branch_count",             in.branch_count },
        { "branch_thread_size_key",   in.branch_thread_size_key },
        { "branch_drill_dia_mm",      bs->drill_dia_mm },
        { "branch_spotface_dia_mm",   bs->spotface_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;drill;bsp_tap";
    tooling.tool_dia_mm       = in.main_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(20.0, 10.0 + in.branch_count * 4.0);
    tooling.extra = {
        { "standard",     "BSPP / ISO 228-1 parallel G-thread port" },
        { "branch_count", in.branch_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::manifold_block_compound: main_dia={} branches={} bsp={}",
                  in.main_dia_mm, in.branch_count, in.branch_thread_size_key);

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

    // Geometric: count main bore (one large cyl) + branch bores (perp cyls).
    int mainCyls = 0;
    int branchCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 6.0) ++mainCyls;
            else ++branchCyls;
        } catch (...) {}
    }
    if (mainCyls >= 1 && branchCyls >= 2) {
        json recovered = { { "branch_count", branchCyls } };
        json matched   = { { "source", "geometric_manifold_pattern" },
                           { "main_cyl_count",   mainCyls },
                           { "branch_cyl_count", branchCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::manifold_block_compound
