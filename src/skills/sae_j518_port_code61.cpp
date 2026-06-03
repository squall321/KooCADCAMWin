// @lat: [[engine/skills#sae_j518_port_code61]]

#include "sae_j518_port_code61.hpp"

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

namespace koocadcam::skill::sae_j518_port_code61 {

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
              "sae_j518_port_code61: depth_mm must be > 0");
    }
    if (in.dash_size.empty()) {
        r.add("DFM-J518C61-CODE", "error",
              "sae_j518_port_code61: dash_size is empty");
        return r;
    }
    const ht::SaeJ518Code61Spec* s = ht::findSaeJ518Code61(in.dash_size);
    if (!s) {
        r.add("DFM-J518C61-CODE", "error",
              "sae_j518_port_code61: unknown dash_size '" + in.dash_size +
              "' (supported: -8, -12, -16, -20, -24, -32)");
        return r;
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-J518C61-FACE", "error",
              "sae_j518_port_code61: face_id datum unresolved");
        return r;
    }
    if (!wp.isFacePlanar(*faceId)) {
        r.add("DFM-J518C61-FACE", "error",
              "sae_j518_port_code61: target face is not planar");
    }

    // Sufficient material around bore: face XY span must be ≥ 1.5 × PCD.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double xSpan = xMax - xMin;
    const double ySpan = yMax - yMin;
    const double minSpan = 1.5 * s->bolt_circle_dia_mm;
    if (xSpan < minSpan || ySpan < minSpan) {
        r.add("DFM-J518C61-MAT", "error",
              "sae_j518_port_code61: face span < 1.5 × bolt_circle (need " +
              std::to_string(minSpan) + " mm)");
    }
    if (zMax - zMin < in.depth_mm + 2.0) {
        r.add("DFM-J518C61-DEPTH", "error",
              "sae_j518_port_code61: face thickness " +
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
        std::string msg = "sae_j518_port_code61 DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::SaeJ518Code61Spec* s = ht::findSaeJ518Code61(in.dash_size);
    if (!s) throw SkillError("sae_j518_port_code61: dash_size lookup failed");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("sae_j518_port_code61: face_id unresolved");

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

    // ── Sub-cut #1: central port through-bore ─────────────────────────────
    const gp_Ax2 boreAx(entryStart, adir);
    const TopoDS_Shape centralBore =
        pr::cylinder(boreAx, portR, in.depth_mm + kOverhang);

    // ── Sub-cut #2: captive face O-ring groove (annular ring) ─────────────
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

    // Pre-fuse all sub-cutters into one and a single Boolean cut.
    TopoDS_Shape unified = pr::fuse(centralBore, oringGroove);
    for (const auto& bh : boltHoles) unified = pr::fuse(unified, bh);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    // Derived volume (sum-of-parts).
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
        { "port_standard",           "SAE_J518_Code61" },
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
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.est_cycle_time_s  = std::max(20.0, in.depth_mm * 0.6 + 10.0);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->port_dia_mm },
              { "role", "central_port_bore" } },
            { { "tool_type", "groove_mill" },
              { "id_dia_mm", s->o_ring_groove_id_mm },
              { "od_dia_mm", s->o_ring_groove_od_mm },
              { "depth_mm", s->o_ring_groove_depth_mm },
              { "role", "captive_face_o_ring_groove" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->bolt_dia_mm },
              { "count", s->bolt_count },
              { "pcd_mm", s->bolt_circle_dia_mm },
              { "role", "bolt_pattern_M" } },
        } },
        { "standard", "SAE J518 Code 61 (3000 psi) 4-bolt split-flange" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::sae_j518_port_code61 applied: {} port={} pcd={}",
                  in.dash_size, s->port_dia_mm, s->bolt_circle_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay first; geometric fallback detects a central bore +
// concentric annular groove + 4 bolt-pattern holes around it.

namespace {

struct CylInfo {
    int    faceIdx;
    gp_Pnt center;
    double radius;
};

std::vector<CylInfo> collectAxialCylinders(const Workpiece& wp, const gp_Dir& adir)
{
    std::vector<CylInfo> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir d = cyl.Axis().Direction();
        const double dot = std::abs(d.X()*adir.X() + d.Y()*adir.Y() + d.Z()*adir.Z());
        if (dot < std::cos(2.0 * M_PI / 180.0)) continue;
        CylInfo ci;
        ci.faceIdx = i;
        ci.center  = cyl.Location();
        ci.radius  = cyl.Radius();
        out.push_back(ci);
    }
    return out;
}

}  // namespace

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

    // Geometric fallback: find a "central bore + 4 bolt holes" cluster.
    const gp_Dir adir(0, 0, 1);
    const auto cyls = collectAxialCylinders(wp, adir);
    if (cyls.size() < 5) return out;

    // Group cylinders by radius; central bore = largest single cyl, bolt holes
    // = 4 cylinders of identical smaller radius arranged on a PCD.
    for (size_t i = 0; i < cyls.size(); ++i) {
        const CylInfo& candCenter = cyls[i];
        std::vector<CylInfo> bolts;
        for (size_t j = 0; j < cyls.size(); ++j) {
            if (j == i) continue;
            if (cyls[j].radius >= candCenter.radius) continue;
            // Distance from candidate center to this cyl's center (XY).
            const double dx = cyls[j].center.X() - candCenter.center.X();
            const double dy = cyls[j].center.Y() - candCenter.center.Y();
            const double dr = std::sqrt(dx * dx + dy * dy);
            if (dr > candCenter.radius * 2.5 && dr < candCenter.radius * 8.0) {
                bolts.push_back(cyls[j]);
            }
        }
        if (bolts.size() < 4) continue;
        // 4 of these bolts should share radius within 0.1 mm.
        const double br = bolts.front().radius;
        int matches = 0;
        for (const auto& b : bolts) if (std::abs(b.radius - br) < 0.1) ++matches;
        if (matches < 4) continue;

        // Map to the closest dash_size by port_dia.
        std::string bestDash = "-12";
        double bestErr = std::numeric_limits<double>::max();
        for (const auto& sp : ht::kSaeJ518Code61) {
            const double err = std::abs(2.0 * candCenter.radius - sp.port_dia_mm);
            if (err < bestErr) { bestErr = err; bestDash = sp.dash_size; }
        }

        RecognizedFeature r;
        r.skill_id = kSkillId;
        r.recovered_params = {
            { "center_x_mm", candCenter.center.X() },
            { "center_y_mm", candCenter.center.Y() },
            { "axis_dir",    { 0.0, 0.0, -1.0 } },
            { "dash_size",   bestDash },
        };
        r.confidence = 0.55;
        r.matched_geometry = {
            { "source",         "geometric_fallback" },
            { "center_face_idx",candCenter.faceIdx },
            { "bolt_count",     matches },
            { "port_dia_mm",    2.0 * candCenter.radius },
        };
        out.push_back(r);
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::sae_j518_port_code61
