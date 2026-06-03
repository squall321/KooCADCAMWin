// @lat: [[engine/skills#bsp_g_thread_port]]

#include "bsp_g_thread_port.hpp"

#include "Workpiece.hpp"
#include "_hydraulic_ports.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::bsp_g_thread_port {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hyd_ports;
using nlohmann::json;

namespace {
constexpr double kOverhang = 0.05;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thread_size.empty()) {
        r.add("DFM-BSPG-CODE", "error",
              "bsp_g_thread_port: thread_size is empty");
        return r;
    }
    const ht::BspGSpec* s = ht::findBspG(in.thread_size);
    if (!s) {
        r.add("DFM-BSPG-CODE", "error",
              "bsp_g_thread_port: unknown thread_size '" + in.thread_size +
              "' (supported: G1/8, G1/4, G3/8, G1/2, G3/4, G1)");
        return r;
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-BSPG-FACE", "error",
              "bsp_g_thread_port: face_id datum unresolved");
        return r;
    }
    if (!wp.isFacePlanar(*faceId)) {
        r.add("DFM-BSPG-FACE", "error",
              "bsp_g_thread_port: target face is not planar");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double minSpan = 1.5 * s->spotface_dia_mm;
    if (xMax - xMin < minSpan || yMax - yMin < minSpan) {
        r.add("DFM-BSPG-MAT", "error",
              "bsp_g_thread_port: face span < 1.5 × spotface (need " +
              std::to_string(minSpan) + " mm)");
    }

    const double depth = (in.depth_override_mm > 0.0) ? in.depth_override_mm
                                                      : s->depth_mm;
    if (zMax - zMin < depth + 2.0) {
        r.add("DFM-BSPG-DEPTH", "error",
              "bsp_g_thread_port: face thickness " +
              std::to_string(zMax - zMin) +
              " mm < depth + 2 mm safety margin");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bsp_g_thread_port DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::BspGSpec* s = ht::findBspG(in.thread_size);
    if (!s) throw SkillError("bsp_g_thread_port: thread_size lookup failed");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("bsp_g_thread_port: face_id unresolved");

    const double depth = (in.depth_override_mm > 0.0) ? in.depth_override_mm
                                                      : s->depth_mm;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir = in.axis_dir;
    gp_Pnt entryStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        entryStart = gp_Pnt(in.center_x_mm, in.center_y_mm,
                            (adir.Z() < 0) ? zMax + kOverhang : zMin - kOverhang);
    } else {
        entryStart = gp_Pnt(in.center_x_mm, in.center_y_mm,
                            (zMin + zMax) / 2.0 - adir.Z() * kOverhang);
    }

    const double drillR    = s->drill_dia_mm    / 2.0;
    const double spotR     = s->spotface_dia_mm / 2.0;
    const double threadR   = s->thread_major_mm / 2.0;
    const double spotDepth = std::max(2.0, s->pitch_mm * 1.5);
    const double chamfLeg  = s->pitch_mm * 0.5;

    // ── Sub-cut #1: spotface (counterbore) at the mouth ──────────────────
    const gp_Ax2 spotAx(entryStart, adir);
    const TopoDS_Shape spotface =
        pr::cylinder(spotAx, spotR, spotDepth + kOverhang);

    // ── Sub-cut #2: main tap-drill bore ──────────────────────────────────
    const gp_Ax2 boreAx(entryStart, adir);
    const TopoDS_Shape mainBore =
        pr::cylinder(boreAx, drillR, depth + kOverhang);

    // ── Sub-cut #3: cosmetic thread groove ──────────────────────────────
    // Approximate the thread root with a slightly enlarged cylinder
    // (drill_dia → thread_minor) below the spotface for the threaded length.
    // No real helix primitive; this is the conventional cosmetic-thread
    // representation used in production STEP exports.
    const double threadMinorR = s->thread_minor_mm / 2.0;
    const gp_Pnt threadStart(entryStart.X() + adir.X() * spotDepth,
                             entryStart.Y() + adir.Y() * spotDepth,
                             entryStart.Z() + adir.Z() * spotDepth);
    const gp_Ax2 threadAx(threadStart, adir);
    const TopoDS_Shape threadGroove =
        pr::cylinder(threadAx, threadMinorR,
                     std::max(0.1, depth - spotDepth) + kOverhang);

    // ── Sub-cut #4: 45° entry chamfer ────────────────────────────────────
    const gp_Ax2 chamfAx(entryStart, adir);
    const TopoDS_Shape chamf = pr::coneFrustum(
        chamfAx,
        spotR + chamfLeg,
        spotR,
        chamfLeg);

    TopoDS_Shape unified = pr::fuse(spotface, mainBore);
    unified = pr::fuse(unified, threadGroove);
    unified = pr::fuse(unified, chamf);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    const double volSpot = M_PI * spotR * spotR * spotDepth;
    const double volBore = M_PI * drillR * drillR * depth;
    const double volThread = std::max(0.0,
        M_PI * (threadMinorR * threadMinorR - drillR * drillR) * (depth - spotDepth));
    const double volChamf = std::max(0.0,
        M_PI / 3.0 * chamfLeg
            * (spotR * spotR + spotR * (spotR + chamfLeg)
               + (spotR + chamfLeg) * (spotR + chamfLeg))
        - M_PI * spotR * spotR * chamfLeg);
    const double volRemoved = volSpot + volBore + volThread + volChamf;

    json params = {
        { "face_id_resolved",   *faceId },
        { "center_x_mm",        in.center_x_mm },
        { "center_y_mm",        in.center_y_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size",        in.thread_size },
        { "depth_override_mm",  in.depth_override_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "port_standard",          "BSPP_ISO_228" },
        { "thread_size",            in.thread_size },
        { "subfeature_count",       4 },
        { "thread_major_mm",        s->thread_major_mm },
        { "thread_minor_mm",        s->thread_minor_mm },
        { "pitch_mm",               s->pitch_mm },
        { "drill_dia_mm",           s->drill_dia_mm },
        { "spotface_dia_mm",        s->spotface_dia_mm },
        { "depth_mm",               depth },
        { "derived_volume_removed", volRemoved },
        { "thread_major_dia_mm",    s->thread_major_mm },
        { "port_dia_mm",            s->drill_dia_mm },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
    };
    (void)threadR;

    ToolingMeta tooling;
    tooling.tool_type         = "counterbore;drill;tap;chamfer_mill";
    tooling.tool_dia_mm       = s->drill_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.est_cycle_time_s  = std::max(10.0, depth * 0.5 + 5.0);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "counterbore" },
              { "tool_dia_mm", s->spotface_dia_mm },
              { "depth_mm", spotDepth },
              { "role", "sealing_spotface" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->drill_dia_mm },
              { "depth_mm", depth },
              { "role", "tap_pilot_bore" } },
            { { "tool_type", "tap" },
              { "thread", in.thread_size },
              { "pitch_mm", s->pitch_mm },
              { "role", "G_parallel_thread" } },
            { { "tool_type", "chamfer_mill" },
              { "leg_mm", chamfLeg },
              { "angle_deg", 45.0 },
              { "role", "thread_start_chamfer" } },
        } },
        { "standard", "BSPP / ISO 228-1 G-thread parallel hydraulic port" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bsp_g_thread_port applied: {} drill={} depth={}",
                  in.thread_size, s->drill_dia_mm, depth);

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
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: detect 2 coaxial cylindrical bores of different
    // radii (spotface + tap drill).  Map drill_dia to nearest table entry.
    struct CylInfo { int idx; gp_Pnt c; double r; };
    std::vector<CylInfo> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        const gp_Dir d = cy.Axis().Direction();
        if (std::abs(d.Z()) < 0.9) continue;
        cyls.push_back({ i, cy.Location(), cy.Radius() });
    }
    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            const double dx = cyls[i].c.X() - cyls[j].c.X();
            const double dy = cyls[i].c.Y() - cyls[j].c.Y();
            if (std::sqrt(dx * dx + dy * dy) > 0.5) continue;
            if (std::abs(cyls[i].r - cyls[j].r) < 0.2) continue;
            const double drillR = std::min(cyls[i].r, cyls[j].r);
            std::string best = "G1/4";
            double bestErr = std::numeric_limits<double>::max();
            for (const auto& sp : ht::kBspG) {
                const double err = std::abs(2.0 * drillR - sp.drill_dia_mm);
                if (err < bestErr) { bestErr = err; best = sp.size; }
            }
            RecognizedFeature r;
            r.skill_id = kSkillId;
            r.recovered_params = {
                { "center_x_mm", cyls[i].c.X() },
                { "center_y_mm", cyls[i].c.Y() },
                { "axis_dir",    { 0.0, 0.0, -1.0 } },
                { "thread_size", best },
            };
            r.confidence = 0.50;
            r.matched_geometry = {
                { "source",       "geometric_fallback" },
                { "drill_dia_mm", 2.0 * drillR },
                { "err_mm",       bestErr },
            };
            out.push_back(r);
            return out;
        }
    }
    return out;
}

}  // namespace koocadcam::skill::bsp_g_thread_port
