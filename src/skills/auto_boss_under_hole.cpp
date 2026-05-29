// @lat: [[engine/skills#auto_boss_under_hole]]

#include "auto_boss_under_hole.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::auto_boss_under_hole {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Lookup table: ISO 4762 socket-head cap screw seat/pilot dims ─────────
namespace {

struct ScrewSpec {
    const char* key;
    double      seat_dia_mm;
    double      seat_depth_mm;
    double      pilot_dia_mm;
};

constexpr std::array<ScrewSpec, 5> kScrewTable {{
    { "M3", 6.0,  3.4, 3.4 },
    { "M4", 8.0,  4.4, 4.5 },
    { "M5", 10.0, 5.4, 5.5 },
    { "M6", 11.0, 6.4, 6.6 },
    { "M8", 15.0, 8.4, 9.0 },
}};

const ScrewSpec* lookupSpec(const std::string& key)
{
    for (const auto& s : kScrewTable)
        if (key == s.key) return &s;
    return nullptr;
}

constexpr double kFloatTol_mm  = 5.0;   // boss synthesised if no solid within 5 mm
constexpr double kMaxFill_mm   = 50.0;  // moulding rule of thumb

// Topology probe: from (x, y, z), walk along axis_dir.  Return the
// projection (in mm) along that axis from the start point at which the
// FIRST piece of solid material is encountered.  If no solid hit, returns
// a sentinel = +infinity.
//
// Slice-1 simplification: we approximate this with the workpiece bbox.
// A real ray cast would call BRepIntersection, but the bbox proxy is
// sufficient for the synthesis decision (do we need a pillar?) — the
// caller compares the returned distance to kFloatTol_mm.
double distanceToFirstSolid(const Workpiece& wp,
                             double startX, double startY, double startZ,
                             const gp_Dir& axisDir)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // If the XY is outside the bbox footprint, no solid below.
    if (startX < xMin - 1e-3 || startX > xMax + 1e-3 ||
        startY < yMin - 1e-3 || startY > yMax + 1e-3) {
        return std::numeric_limits<double>::infinity();
    }

    // For axisDir == -Z (typical), distance to first solid is startZ − zMax
    // (top of bbox); negative ⇒ already inside.
    if (std::abs(axisDir.X()) < 1e-6 && std::abs(axisDir.Y()) < 1e-6) {
        if (axisDir.Z() < 0) {
            return std::max(0.0, startZ - zMax);
        } else {
            return std::max(0.0, zMin - startZ);
        }
    }
    // Generic non-axial: bbox proxy degenerates — return 0 (assume solid).
    return 0.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.boss_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "boss_dia_mm must be > 0");
    }
    if (in.total_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "total_depth_mm must be > 0");
    }
    const ScrewSpec* spec = lookupSpec(in.screw_spec);
    if (!spec) {
        r.add("DFM-BOSS-SIZE", "error",
              "auto_boss_under_hole: unknown screw_spec '" + in.screw_spec +
              "' (supported: M3/M4/M5/M6/M8)");
        return r;
    }

    // Wall thickness: boss_dia must exceed seat_dia by at least 2 mm
    // (wall thickness >= 1 mm).  Reference: standard injection-mould
    // boss-thickness rule.
    if (in.boss_dia_mm < spec->seat_dia_mm + 2.0) {
        r.add("DFM-BOSS-WALL", "error",
              "boss_dia " + std::to_string(in.boss_dia_mm) +
              " < seat_dia (" + std::to_string(spec->seat_dia_mm) +
              ") + 2 mm wall");
    }

    // Probe how much fill pillar would be added.
    const double dist = distanceToFirstSolid(wp,
        in.position_x_mm, in.position_y_mm, in.start_z_mm, in.axis_dir);
    if (std::isfinite(dist) && dist > kMaxFill_mm) {
        r.add("DFM-BOSS-DEPTH", "warning",
              "synthesised pillar fill " + std::to_string(dist) +
              " mm exceeds mouldability max " + std::to_string(kMaxFill_mm));
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "auto_boss_under_hole DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ScrewSpec* spec = lookupSpec(in.screw_spec);
    // safe: validate() returned passed=true → spec is non-null.

    // ── Probe topology ──────────────────────────────────────────────────
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dist = distanceToFirstSolid(wp,
        in.position_x_mm, in.position_y_mm, in.start_z_mm, in.axis_dir);

    const bool needPillar = std::isfinite(dist) && dist > kFloatTol_mm;
    const double kOver = 0.1;

    const gp_Dir adir = in.axis_dir;
    const double r_boss = in.boss_dia_mm / 2.0;
    const double r_seat = spec->seat_dia_mm / 2.0;
    const double r_pilot = spec->pilot_dia_mm / 2.0;

    TopoDS_Shape working = wp.shape();

    // ── Sub-feature 1: pillar fill (fuse), only if floating ─────────────
    double pillarHeight = 0.0;
    double pillarTopZ   = in.start_z_mm;
    double pillarBotZ   = in.start_z_mm;
    if (needPillar) {
        // Walk down from start_z by `dist` mm — fuse a cylinder from
        // (start_z) down to (start_z − dist).
        pillarHeight = dist;
        // Pillar grows along -axisDir (i.e. starts at top, extends down).
        // Build with gp_Ax2 at pillar BOTTOM and Z direction = -axisDir
        // (so the cylinder grows upward to the start point).
        const gp_Dir pillarUp(-adir.X(), -adir.Y(), -adir.Z());
        const gp_Pnt pillarBottom(
            in.position_x_mm + adir.X() * pillarHeight,
            in.position_y_mm + adir.Y() * pillarHeight,
            in.start_z_mm    + adir.Z() * pillarHeight);
        pillarBotZ = pillarBottom.Z();
        pillarTopZ = in.start_z_mm;
        const gp_Ax2 axPillar(pillarBottom, pillarUp);
        const TopoDS_Shape pillar = pr::cylinder(axPillar, r_boss, pillarHeight);
        working = pr::fuse(working, pillar);
    }

    // ── Sub-feature 2 + 3: fuse(seat_cyl, pilot_cyl) then cut once ─────
    //
    // Per project rule: "For overlapping concentric cylinders: pr::fuse
    // then pr::cut" — counterbore seat (large dia, shallow) and pilot
    // (small dia, deep) are concentric, so fuse the cutters first.
    const gp_Pnt cutStart(
        in.position_x_mm - adir.X() * kOver,
        in.position_y_mm - adir.Y() * kOver,
        in.start_z_mm    - adir.Z() * kOver);
    const gp_Ax2 axCut(cutStart, adir);

    const TopoDS_Shape seatTool = pr::cylinder(
        axCut, r_seat, spec->seat_depth_mm + kOver);
    const TopoDS_Shape pilotTool = pr::cylinder(
        axCut, r_pilot, in.total_depth_mm + kOver);
    const TopoDS_Shape fusedCutter = pr::fuse(seatTool, pilotTool);
    const TopoDS_Shape newShape = pr::cut(working, fusedCutter);

    // ── Bookkeeping ─────────────────────────────────────────────────────
    const double vPillar = needPillar
        ? M_PI * r_boss * r_boss * pillarHeight
        : 0.0;
    const double vSeat  = M_PI * r_seat * r_seat * spec->seat_depth_mm;
    const double vPilot = M_PI * r_pilot * r_pilot *
                          std::max(0.0, in.total_depth_mm - spec->seat_depth_mm);

    json params = {
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "start_z_mm",     in.start_z_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "boss_dia_mm",    in.boss_dia_mm },
        { "screw_spec",     in.screw_spec },
        { "total_depth_mm", in.total_depth_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  needPillar ? 3 : 2 },
        { "pillar_synthesized",needPillar },
        { "pillar_height_mm",  pillarHeight },
        { "screw_spec",        in.screw_spec },
        { "seat_dia_mm",       spec->seat_dia_mm },
        { "pilot_dia_mm",      spec->pilot_dia_mm },
        { "boss_dia_mm",       in.boss_dia_mm },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
        { "standard",          "ISO 4762" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = needPillar ? "mould;drill;counterbore"
                                            : "drill;counterbore";
    tooling.tool_dia_mm       = spec->seat_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.stock_removed_mm3 = vSeat + vPilot;
    tooling.est_cycle_time_s  = 45.0 + (needPillar ? 30.0 : 0.0);
    tooling.extra = {
        { "pillar_volume_mm3",  vPillar },
        { "seat_volume_mm3",    vSeat },
        { "pilot_volume_mm3",   vPilot },
        { "added_volume_mm3",   vPillar },
        { "removed_volume_mm3", vSeat + vPilot },
        { "net_delta_mm3",      vPillar - (vSeat + vPilot) },
        { "standard",           "ISO 4762" },
        { "pillar_top_z_mm",    pillarTopZ },
        { "pillar_bot_z_mm",    pillarBotZ },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::auto_boss_under_hole: pillar={} spec={} bossDia={}",
                  needPillar, in.screw_spec, in.boss_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

struct CylRec { int idx; double radius; gp_Ax1 axis; };

std::vector<CylRec> collectCyls(const Workpiece& wp)
{
    std::vector<CylRec> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        out.push_back({ i, s.Cylinder().Radius(), s.Cylinder().Axis() });
    }
    return out;
}

bool axesParallel(const gp_Ax1& a, const gp_Ax1& b)
{
    const double dot = std::abs(
        a.Direction().X() * b.Direction().X() +
        a.Direction().Y() * b.Direction().Y() +
        a.Direction().Z() * b.Direction().Z());
    return dot > 0.999;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    auto emitMetadataReplay = [&]() {
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            RecognizedFeature rf;
            rf.skill_id = kSkillId;
            rf.recovered_params = f.params;
            rf.confidence = 1.0;
            rf.matched_geometry = { { "via", "metadata_replay" } };
            out.push_back(rf);
            return;
        }
    };

    const auto cyls = collectCyls(wp);
    if (cyls.size() < 2) {
        emitMetadataReplay();
        return out;
    }

    // Look for a SEAT/PILOT concentric pair whose dimensions match the
    // lookup table — that's our geometric fingerprint for the
    // counterbore-plus-pilot signature this skill leaves.
    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (!axesParallel(cyls[i].axis, cyls[j].axis)) continue;
            const CylRec& small = (cyls[i].radius < cyls[j].radius) ? cyls[i] : cyls[j];
            const CylRec& large = (cyls[i].radius < cyls[j].radius) ? cyls[j] : cyls[i];
            if (std::abs(large.radius - small.radius) < 0.1) continue;

            // Check against table.
            const std::string keyGuess = [&]() -> std::string {
                for (const auto& s : kScrewTable) {
                    if (std::abs(2.0 * large.radius - s.seat_dia_mm) < 0.5 &&
                        std::abs(2.0 * small.radius - s.pilot_dia_mm) < 0.5) {
                        return s.key;
                    }
                }
                return {};
            }();
            if (keyGuess.empty()) continue;

            json recovered = {
                { "position_x_mm", small.axis.Location().X() },
                { "position_y_mm", small.axis.Location().Y() },
                { "axis_dir",      { small.axis.Direction().X(),
                                     small.axis.Direction().Y(),
                                     small.axis.Direction().Z() } },
                { "screw_spec",    keyGuess },
                { "boss_dia_mm",   2.0 * large.radius + 2.0 },
                { "total_depth_mm", 8.0 },
            };
            json matched = {
                { "seat_face_id",  large.idx },
                { "pilot_face_id", small.idx },
                { "via",           "geometric_primary" },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.8, matched });
        }
    }

    if (out.empty()) emitMetadataReplay();
    return out;
}

}  // namespace koocadcam::skill::auto_boss_under_hole
