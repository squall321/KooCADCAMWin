// @lat: [[engine/skills#banjo_fitting_seat]]

#include "banjo_fitting_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace koocadcam::skill::banjo_fitting_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct BoltEntry {
    const char* size;
    double      clearance_dia_mm;   // close-fit clearance hole (ISO 273)
    double      washer_seat_dia_mm; // DIN 7603/7642 banjo washer OD
};

constexpr std::array<BoltEntry, 5> kBoltTable {{
    { "M6",   6.6,  10.0 },
    { "M8",   9.0,  14.0 },
    { "M10", 11.0,  17.0 },
    { "M12", 13.5,  19.0 },
    { "M14", 15.5,  21.0 },
}};

const BoltEntry* find(const std::string& s)
{
    for (const auto& e : kBoltTable) if (s == e.size) return &e;
    return nullptr;
}

// Build any unit vector perpendicular to `axis` in a stable way.
gp_Dir perpendicularTo(const gp_Dir& axis)
{
    const gp_Dir seed = (std::abs(axis.Z()) < 0.9) ? gp_Dir(0, 0, 1) : gp_Dir(1, 0, 0);
    gp_Vec vx = gp_Vec(seed).Crossed(gp_Vec(axis));
    if (vx.Magnitude() < 1e-9) vx = gp_Vec(1, 0, 0);
    return gp_Dir(vx);
}

}  // namespace

bool resolveBoltM(const std::string& size,
                  double& clearance_dia_mm,
                  double& washer_seat_dia_mm)
{
    const BoltEntry* e = find(size);
    if (!e) return false;
    clearance_dia_mm   = e->clearance_dia_mm;
    washer_seat_dia_mm = e->washer_seat_dia_mm;
    return true;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.banjo_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "banjo_fitting_seat: banjo_dia_mm must be > 0");
    }
    if (in.washer_seat_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "banjo_fitting_seat: washer_seat_depth_mm must be > 0");
    }
    if (in.part_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "banjo_fitting_seat: part_thickness_mm must be > 0");
    }

    if (in.banjo_dia_mm > 0.0 && in.banjo_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "banjo_fitting_seat: banjo_dia " + std::to_string(in.banjo_dia_mm) +
              " mm < min 0.8 mm");
    }

    if (in.bolt_M.empty()) {
        r.add("DFM-INPUT", "error",
              "banjo_fitting_seat: bolt_M is empty");
        return r;
    }
    const BoltEntry* e = find(in.bolt_M);
    if (!e) {
        r.add("DFM-INPUT", "error",
              "banjo_fitting_seat: unknown bolt_M '" + in.bolt_M +
              "' (supported: M6, M8, M10, M12, M14)");
        return r;
    }

    if (e->clearance_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "banjo_fitting_seat: bolt clearance dia < 0.8 mm");
    }

    // Washer seat must offer real seating annulus around the bolt.
    if (e->washer_seat_dia_mm <= e->clearance_dia_mm + 2.0) {
        r.add("DFM-BANJO-SEAT", "error",
              "banjo_fitting_seat: washer_seat_dia " +
              std::to_string(e->washer_seat_dia_mm) +
              " mm too close to bolt clearance " +
              std::to_string(e->clearance_dia_mm) +
              " mm (no metal under washer)");
    }

    // The cross-drill must NOT be larger than the bolt clearance — otherwise
    // it would sever the bolt entirely.  Convention: passage <= 0.6 × bolt.
    if (in.banjo_dia_mm >= e->clearance_dia_mm) {
        r.add("DFM-BANJO-FLOW", "error",
              "banjo_fitting_seat: banjo_dia " + std::to_string(in.banjo_dia_mm) +
              " mm >= bolt clearance " + std::to_string(e->clearance_dia_mm) +
              " mm (cross-drill would sever bolt)");
    }

    // Two opposite washer cbores must not overlap.
    if (2.0 * in.washer_seat_depth_mm + 0.5 >= in.part_thickness_mm) {
        r.add("DFM-BANJO-GEOM", "error",
              "banjo_fitting_seat: both washer cbores would meet inside the part "
              "(2 × seat_depth >= part_thickness - 0.5 mm web)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "banjo_fitting_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const BoltEntry* e = find(in.bolt_M);
    if (!e) throw SkillError("banjo_fitting_seat: bolt_M lookup failed unexpectedly");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("banjo_fitting_seat: face_id datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    constexpr double kOverhang = 0.05;
    const gp_Dir bdir = in.bolt_axis;
    const double boltR  = e->clearance_dia_mm   / 2.0;
    const double seatR  = e->washer_seat_dia_mm / 2.0;
    const double crossR = in.banjo_dia_mm       / 2.0;

    // ── Sub-cut #1: through-hole at bolt clearance dia ──────────────────
    // Start outside the workpiece along -bdir to ensure clean through.
    gp_Pnt boltStart(in.position_x_mm, in.position_y_mm, 0.0);
    if (std::abs(bdir.X()) < 1e-6 && std::abs(bdir.Y()) < 1e-6) {
        boltStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                           (bdir.Z() < 0) ? zMax + kOverhang : zMin - kOverhang);
    } else {
        boltStart = gp_Pnt(
            in.position_x_mm - bdir.X() * (bboxDiag + kOverhang),
            in.position_y_mm - bdir.Y() * (bboxDiag + kOverhang),
            (zMin + zMax) / 2.0 - bdir.Z() * (bboxDiag + kOverhang));
    }
    const double throughLen = bboxDiag + 2.0 * kOverhang;
    const gp_Ax2 boltAx(boltStart, bdir);
    const TopoDS_Shape boltHole = pr::cylinder(boltAx, boltR, throughLen);

    // ── Sub-cut #2: TOP-face washer counterbore ─────────────────────────
    // Same axis as the bolt, starts at the +Z face (= boltStart), depth =
    // washer_seat_depth.
    const gp_Ax2 cb1Ax(boltStart, bdir);
    const TopoDS_Shape cbTop = pr::cylinder(
        cb1Ax, seatR, in.washer_seat_depth_mm + kOverhang);

    // ── Sub-cut #3: BOTTOM-face washer counterbore ──────────────────────
    // Reversed axis (= -bdir), starts at the opposite face.  For an
    // axis-aligned cuboid: opposite face is at zMin (if bdir = -Z).
    gp_Pnt botStart;
    gp_Dir botAxis(-bdir.X(), -bdir.Y(), -bdir.Z());
    if (std::abs(bdir.X()) < 1e-6 && std::abs(bdir.Y()) < 1e-6) {
        botStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                          (bdir.Z() < 0) ? zMin - kOverhang : zMax + kOverhang);
    } else {
        // Walk to the opposite side: travel +bdir from boltStart by
        // (throughLen - overhang).
        botStart = gp_Pnt(
            boltStart.X() + bdir.X() * (throughLen - kOverhang),
            boltStart.Y() + bdir.Y() * (throughLen - kOverhang),
            boltStart.Z() + bdir.Z() * (throughLen - kOverhang));
    }
    const gp_Ax2 cb2Ax(botStart, botAxis);
    const TopoDS_Shape cbBot = pr::cylinder(
        cb2Ax, seatR, in.washer_seat_depth_mm + kOverhang);

    // ── Sub-cut #4: cross-drilled fluid passage ─────────────────────────
    // Perpendicular to bdir, passing through the bolt axis at axial offset
    // `banjo_axial_z_mm`.  Walk back along a perpendicular direction from
    // the bolt axis sample point, and cut a long cylinder across the part.
    const gp_Dir crossDir = perpendicularTo(bdir);
    // Sample point on bolt axis at the requested axial offset (relative
    // to the bolt entry).
    const gp_Pnt crossCenter(
        in.position_x_mm + bdir.X() * in.banjo_axial_z_mm,
        in.position_y_mm + bdir.Y() * in.banjo_axial_z_mm,
        boltStart.Z()    + bdir.Z() * in.banjo_axial_z_mm);
    const gp_Pnt crossStart(
        crossCenter.X() - crossDir.X() * (bboxDiag + kOverhang),
        crossCenter.Y() - crossDir.Y() * (bboxDiag + kOverhang),
        crossCenter.Z() - crossDir.Z() * (bboxDiag + kOverhang));
    const gp_Ax2 crossAx(crossStart, crossDir);
    const TopoDS_Shape crossPass = pr::cylinder(
        crossAx, crossR, throughLen);

    // Pre-fuse concentric cutters along the bolt axis (boltHole + cbTop +
    // cbBot all share the same line, with overlapping pieces near each
    // face).  Then add the cross passage.
    TopoDS_Shape boltStack = pr::fuse(boltHole, cbTop);
    boltStack = pr::fuse(boltStack, cbBot);
    TopoDS_Shape unified = pr::fuse(boltStack, crossPass);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "bolt_axis",            { bdir.X(), bdir.Y(), bdir.Z() } },
        { "bolt_M",               in.bolt_M },
        { "banjo_dia_mm",         in.banjo_dia_mm },
        { "banjo_axial_z_mm",     in.banjo_axial_z_mm },
        { "washer_seat_depth_mm", in.washer_seat_depth_mm },
        { "part_thickness_mm",    in.part_thickness_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      4 },          // bolt + 2 cbore + cross
        { "bolt_M",                in.bolt_M },
        { "bolt_clearance_dia_mm", e->clearance_dia_mm },             // derived
        { "washer_seat_dia_mm",    e->washer_seat_dia_mm },           // derived
        { "washer_seat_depth_mm",  in.washer_seat_depth_mm },
        { "banjo_dia_mm",          in.banjo_dia_mm },
        { "cross_dir",             { crossDir.X(), crossDir.Y(), crossDir.Z() } },
        { "bolt_axis",             { bdir.X(), bdir.Y(), bdir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;counterbore;counterbore;drill";
    tooling.tool_dia_mm       = e->clearance_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.est_cycle_time_s  = std::max(3.0, in.part_thickness_mm * 0.3 + 2.0);
    const double boltVol  = M_PI * boltR * boltR * in.part_thickness_mm;
    const double cboresVol = 2.0 * (M_PI * seatR * seatR - M_PI * boltR * boltR)
                           * in.washer_seat_depth_mm;
    const double crossVol = M_PI * crossR * crossR * in.part_thickness_mm;
    tooling.stock_removed_mm3 = boltVol + std::max(0.0, cboresVol) + crossVol;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", e->clearance_dia_mm },
              { "role", "bolt_through" } },
            { { "tool_type", "counterbore" },
              { "tool_dia_mm", e->washer_seat_dia_mm },
              { "depth_mm", in.washer_seat_depth_mm },
              { "role", "top_washer" } },
            { { "tool_type", "counterbore" },
              { "tool_dia_mm", e->washer_seat_dia_mm },
              { "depth_mm", in.washer_seat_depth_mm },
              { "role", "bottom_washer" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.banjo_dia_mm },
              { "role", "cross_fluid_passage" } },
        } },
        { "note", "banjo / hollow-bolt fluid coupling per DIN 7643 / SAE J1467" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::banjo_fitting_seat applied: bolt={} clearance={} banjo={} part_t={}",
                  in.bolt_M, e->clearance_dia_mm, in.banjo_dia_mm, in.part_thickness_mm);

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

}  // namespace koocadcam::skill::banjo_fitting_seat
