// @lat: [[engine/skills#sae_j518_port]]

#include "sae_j518_port.hpp"

#include "Workpiece.hpp"
#include "_hydraulic_ports_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace koocadcam::skill::sae_j518_port {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hydraulic_ports;
using nlohmann::json;

namespace {

constexpr double kOverhang = 0.05;

// Build any unit vector perpendicular to `axis` in a stable way.
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
              "sae_j518_port: depth_mm must be > 0");
    }
    if (in.sae_code.empty()) {
        r.add("DFM-J518-CODE", "error", "sae_j518_port: sae_code is empty");
        return r;
    }
    const ht::SaeJ518Spec* s = ht::findSaeJ518(in.sae_code);
    if (!s) {
        r.add("DFM-J518-CODE", "error",
              "sae_j518_port: unknown sae_code '" + in.sae_code +
              "' (supported: -08, -12, -16, -24)");
        return r;
    }

    // Face thickness >= depth + 2 mm (port bottom needs material backing).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zSpan = zMax - zMin;
    const double xSpan = xMax - xMin;
    const double ySpan = yMax - yMin;
    if (zSpan < in.depth_mm + 2.0) {
        r.add("DFM-J518-DEPTH", "error",
              "sae_j518_port: face thickness " + std::to_string(zSpan) +
              " mm < depth + 2 mm safety (" + std::to_string(in.depth_mm + 2.0) + ")");
    }

    // Position inside the face bbox with margin = PCD/2 + bolt_dia/2.
    const double margin = s->pcd_mm / 2.0 + s->bolt_dia_mm / 2.0 + 1.0;
    if (in.position_x_mm - xMin < margin || xMax - in.position_x_mm < margin ||
        in.position_y_mm - yMin < margin || yMax - in.position_y_mm < margin)
    {
        r.add("DFM-J518-XY", "error",
              "sae_j518_port: position (" + std::to_string(in.position_x_mm) +
              ", " + std::to_string(in.position_y_mm) +
              ") within " + std::to_string(margin) +
              " mm of face edge (bolt pattern would overflow)");
    }
    (void)xSpan; (void)ySpan;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sae_j518_port DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::SaeJ518Spec* s = ht::findSaeJ518(in.sae_code);
    if (!s) throw SkillError("sae_j518_port: sae_code lookup failed unexpectedly");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("sae_j518_port: face_id datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir = in.axis_dir;
    // Entry point on the +adir face.
    gp_Pnt entryStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        entryStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                            (adir.Z() < 0) ? zMax + kOverhang : zMin - kOverhang);
    } else {
        entryStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                            (zMin + zMax) / 2.0 - adir.Z() * kOverhang);
    }

    const double thruR  = s->thru_dia_mm / 2.0;
    const double boltR  = s->bolt_dia_mm / 2.0;
    const double pcdR   = s->pcd_mm      / 2.0;
    const double oringR = s->oring_id_mm / 2.0;
    const double cs     = s->oring_cs_mm;

    // ── Sub-cut #1: central through / port bore ───────────────────────────
    const gp_Ax2 boreAx(entryStart, adir);
    const TopoDS_Shape centralBore =
        pr::cylinder(boreAx, thruR, in.depth_mm + kOverhang);

    // ── Sub-cut #2: 4 bolt-pattern holes on PCD ───────────────────────────
    // PCD is in the face plane (perpendicular to adir).
    const gp_Dir uAx = perpendicularTo(adir);
    const gp_Vec vAx = gp_Vec(adir).Crossed(gp_Vec(uAx));   // already unit
    std::array<TopoDS_Shape, 4> boltHoles;
    for (int k = 0; k < 4; ++k) {
        const double th = M_PI / 4.0 + k * M_PI / 2.0;        // 45°, 135°, 225°, 315°
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

    // ── Sub-cut #3: captive face O-ring groove (annular ring on the face) ─
    // Groove is concentric with the central bore; ID ≈ thruR, OD = ID + cs
    // (radial groove width = cross-section).  Depth ≈ 0.75 × cs.
    const double grooveID   = oringR;                  // inner radius
    const double grooveOD   = oringR + cs;             // outer radius
    const double grooveD    = 0.75 * cs;
    // Place groove just below the face so the boolean cut exposes it on the face.
    gp_Pnt grooveStart(entryStart);
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        grooveStart = gp_Pnt(in.position_x_mm, in.position_y_mm, entryStart.Z());
    }
    const gp_Ax2 grooveAx(grooveStart, adir);
    const TopoDS_Shape oringGroove =
        pr::annularRing(grooveAx, grooveOD, grooveID, grooveD + kOverhang);

    // ── Sub-cut #4: entry chamfer on central bore (45° leg) ───────────────
    const double chamfLeg = std::max(0.3, cs * 0.5);
    const gp_Ax2 chamfAx(entryStart, adir);
    const TopoDS_Shape chamf = pr::coneFrustum(
        chamfAx,
        thruR + chamfLeg,                 // wide at face
        thruR,                            // narrows to bore wall
        chamfLeg);

    // Pre-fuse all sub-cutters into one then a single Boolean cut.
    TopoDS_Shape unified = pr::fuse(centralBore, chamf);
    unified = pr::fuse(unified, oringGroove);
    for (const auto& bh : boltHoles) unified = pr::fuse(unified, bh);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    // ── Derived volume removed (sum-of-parts, for tests / tooling) ──────
    const double volBore   = M_PI * thruR * thruR * in.depth_mm;
    const double volBolts  = 4.0 * M_PI * boltR * boltR * in.depth_mm;
    const double volGroove = M_PI * (grooveOD * grooveOD - grooveID * grooveID) * grooveD;
    const double volChamf  = M_PI / 3.0 * chamfLeg
                           * (thruR * thruR + thruR * (thruR + chamfLeg)
                              + (thruR + chamfLeg) * (thruR + chamfLeg))
                           - M_PI * thruR * thruR * chamfLeg;
    const double volRemoved = volBore + volBolts + volGroove + std::max(0.0, volChamf);

    json params = {
        { "face_id_resolved", *faceId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "sae_code",         in.sae_code },
        { "depth_mm",         in.depth_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "port_standard",           "SAE_J518" },
        { "size_code",               in.sae_code },
        { "subfeature_count",        4 + 4 - 1 },          // bore + groove + chamf + 4 bolts
        { "thru_dia_mm",             s->thru_dia_mm },
        { "pcd_mm",                  s->pcd_mm },
        { "bolt_dia_mm",             s->bolt_dia_mm },
        { "oring_id_mm",             s->oring_id_mm },
        { "oring_cs_mm",             s->oring_cs_mm },
        { "derived_volume_removed",  volRemoved },
        { "axis_dir",                { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;drill;groove_mill;chamfer_mill";
    tooling.tool_dia_mm       = s->thru_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.est_cycle_time_s  = std::max(15.0, in.depth_mm * 0.5 + 8.0);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->thru_dia_mm },
              { "role", "central_port_bore" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", s->bolt_dia_mm },
              { "count", 4 },
              { "pcd_mm", s->pcd_mm },
              { "role", "bolt_pattern" } },
            { { "tool_type", "groove_mill" },
              { "id_dia_mm", 2.0 * grooveID },
              { "od_dia_mm", 2.0 * grooveOD },
              { "depth_mm", grooveD },
              { "role", "face_o_ring_groove" } },
            { { "tool_type", "chamfer_mill" },
              { "leg_mm",    chamfLeg },
              { "angle_deg", 45.0 },
              { "role",      "bore_entry_chamfer" } },
        } },
        { "note", "SAE J518 Code 61 4-bolt split-flange port" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::sae_j518_port applied: {} thru={} pcd={} depth={}",
                  in.sae_code, s->thru_dia_mm, s->pcd_mm, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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

}  // namespace koocadcam::skill::sae_j518_port
