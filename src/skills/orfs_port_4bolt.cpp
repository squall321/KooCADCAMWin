// @lat: [[engine/skills#orfs_port_4bolt]]

#include "orfs_port_4bolt.hpp"

#include "Workpiece.hpp"
#include "_hydraulic_ports.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::orfs_port_4bolt {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hyd_ports;
using nlohmann::json;

namespace {

constexpr double kOverhang = 0.05;

gp_Dir perpendicularTo(const gp_Dir& axis)
{
    const gp_Dir seed = (std::abs(axis.Z()) < 0.9) ? gp_Dir(0, 0, 1) : gp_Dir(1, 0, 0);
    gp_Vec vx = gp_Vec(seed).Crossed(gp_Vec(axis));
    if (vx.Magnitude() < 1e-9) vx = gp_Vec(1, 0, 0);
    return gp_Dir(vx);
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "orfs_port_4bolt: depth_mm must be > 0");
    }
    if (in.dash_size.empty()) {
        r.add("DFM-ORFS-CODE", "error",
              "orfs_port_4bolt: dash_size is empty");
        return r;
    }
    const ht::OrfsPortSpec* s = ht::findOrfsPort(in.dash_size);
    if (!s) {
        r.add("DFM-ORFS-CODE", "error",
              "orfs_port_4bolt: unknown dash_size '" + in.dash_size +
              "' (supported: -4, -6, -8, -12, -16, -20)");
        return r;
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-ORFS-FACE", "error",
              "orfs_port_4bolt: face_id datum unresolved");
        return r;
    }
    if (!wp.isFacePlanar(*faceId)) {
        r.add("DFM-ORFS-FACE", "error",
              "orfs_port_4bolt: target face is not planar");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double minSpan = 1.5 * s->bolt_circle_dia_mm;
    if (xMax - xMin < minSpan || yMax - yMin < minSpan) {
        r.add("DFM-ORFS-MAT", "error",
              "orfs_port_4bolt: face span < 1.5 × bolt_circle (need " +
              std::to_string(minSpan) + " mm)");
    }
    if (zMax - zMin < in.depth_mm + 2.0) {
        r.add("DFM-ORFS-DEPTH", "error",
              "orfs_port_4bolt: face thickness " +
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
        std::string msg = "orfs_port_4bolt DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::OrfsPortSpec* s = ht::findOrfsPort(in.dash_size);
    if (!s) throw SkillError("orfs_port_4bolt: dash_size lookup failed");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("orfs_port_4bolt: face_id unresolved");

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

    const double portR  = s->port_dia_mm           / 2.0;
    const double boltR  = s->bolt_dia_mm           / 2.0;
    const double pcdR   = s->bolt_circle_dia_mm    / 2.0;
    const double grvIDR = s->o_ring_groove_id_mm   / 2.0;
    const double grvODR = s->o_ring_groove_od_mm   / 2.0;
    const double grvD   = s->o_ring_groove_depth_mm;

    // ── Sub-cut #1: central port bore ─────────────────────────────────────
    const gp_Ax2 boreAx(entryStart, adir);
    const TopoDS_Shape centralBore =
        pr::cylinder(boreAx, portR, in.depth_mm + kOverhang);

    // ── Sub-cut #2: concentric face O-ring groove ─────────────────────────
    const gp_Ax2 grooveAx(entryStart, adir);
    const TopoDS_Shape oringGroove =
        pr::annularRing(grooveAx, grvODR, grvIDR, grvD + kOverhang);

    // ── Sub-cut #3..6: 4 bolt-clearance holes on PCD ──────────────────────
    const gp_Dir uAx = perpendicularTo(adir);
    const gp_Vec vAx = gp_Vec(adir).Crossed(gp_Vec(uAx));
    std::array<TopoDS_Shape, 4> boltHoles;
    for (int k = 0; k < 4; ++k) {
        const double th = M_PI / 4.0 + k * M_PI / 2.0;
        const double dx = pcdR * (uAx.X() * std::cos(th) + vAx.X() * std::sin(th));
        const double dy = pcdR * (uAx.Y() * std::cos(th) + vAx.Y() * std::sin(th));
        const double dz = pcdR * (uAx.Z() * std::cos(th) + vAx.Z() * std::sin(th));
        const gp_Pnt boltStart(entryStart.X() + dx,
                               entryStart.Y() + dy,
                               entryStart.Z() + dz);
        const gp_Ax2 boltAx(boltStart, adir);
        boltHoles[static_cast<std::size_t>(k)] =
            pr::cylinder(boltAx, boltR, in.depth_mm + kOverhang);
    }

    TopoDS_Shape unified = pr::fuse(centralBore, oringGroove);
    for (const auto& bh : boltHoles) unified = pr::fuse(unified, bh);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    const double volBore   = M_PI * portR * portR * in.depth_mm;
    const double volBolts  = 4.0 * M_PI * boltR * boltR * in.depth_mm;
    const double volGroove = M_PI * (grvODR * grvODR - grvIDR * grvIDR) * grvD;
    const double volRemoved = volBore + volBolts + volGroove;

    json params = {
        { "face_id_resolved", *faceId },
        { "center_x_mm",      in.center_x_mm },
        { "center_y_mm",      in.center_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "dash_size",        in.dash_size },
        { "depth_mm",         in.depth_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "port_standard",           "ORFS_4bolt_SAE_J1453" },
        { "dash_size",               in.dash_size },
        { "subfeature_count",        2 + s->bolt_count },
        { "port_dia_mm",             s->port_dia_mm },
        { "bolt_circle_dia_mm",      s->bolt_circle_dia_mm },
        { "bolt_dia_mm",             s->bolt_dia_mm },
        { "bolt_count",              s->bolt_count },
        { "o_ring_groove_id_mm",     s->o_ring_groove_id_mm },
        { "o_ring_groove_od_mm",     s->o_ring_groove_od_mm },
        { "o_ring_groove_depth_mm",  s->o_ring_groove_depth_mm },
        { "derived_volume_removed",  volRemoved },
        { "axis_dir",                { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;groove_mill;drill";
    tooling.tool_dia_mm       = s->port_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 240.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.est_cycle_time_s  = std::max(16.0, in.depth_mm * 0.6 + 8.0);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->port_dia_mm },
              { "role", "central_port_bore" } },
            { { "tool_type", "groove_mill" },
              { "id_dia_mm", s->o_ring_groove_id_mm },
              { "od_dia_mm", s->o_ring_groove_od_mm },
              { "depth_mm",  s->o_ring_groove_depth_mm },
              { "role", "captive_face_o_ring_groove" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->bolt_dia_mm },
              { "count", s->bolt_count },
              { "pcd_mm", s->bolt_circle_dia_mm },
              { "role", "bolt_pattern_M" } },
        } },
        { "standard", "ORFS 4-bolt face-seal port (SAE J1453 flange variant)" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::orfs_port_4bolt applied: {} port={} pcd={}",
                  in.dash_size, s->port_dia_mm, s->bolt_circle_dia_mm);

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
    return out;
}

}  // namespace koocadcam::skill::orfs_port_4bolt
