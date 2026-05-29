// @lat: [[engine/skills#auto_standoff_floating_point]]

#include "auto_standoff_floating_point.hpp"

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

namespace koocadcam::skill::auto_standoff_floating_point {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Lookup table: PCB-mount metric tapped standoff specs ─────────────────
namespace {

struct StandoffSpec {
    const char* key;
    double      pilot_dia_mm;
    double      min_od_mm;
    double      min_wall_mm;
};

constexpr std::array<StandoffSpec, 6> kStandoffTable {{
    { "M2",   1.6,  4.0,  1.2  },
    { "M2.5", 2.05, 5.0,  1.5  },
    { "M3",   2.5,  6.0,  1.75 },
    { "M4",   3.3,  8.0,  2.0  },
    { "M5",   4.2,  10.0, 2.5  },
    { "M6",   5.0,  12.0, 3.0  },
}};

const StandoffSpec* lookupSpec(const std::string& key)
{
    for (const auto& s : kStandoffTable)
        if (key == s.key) return &s;
    return nullptr;
}

// Detect whether the workpiece already has solid material near Z = 0
// covering the XY position (i.e. a baseplate is present at the floor).
// Slice-1 proxy: workpiece's bbox includes Z = 0 within tol AND the XY
// is inside the bbox.
bool baseAtZeroPresent(const Workpiece& wp, double x, double y, double zTol)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    if (x < xMin - 1e-3 || x > xMax + 1e-3) return false;
    if (y < yMin - 1e-3 || y > yMax + 1e-3) return false;
    return zMin <= zTol && zMax >= -zTol;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "height_mm must be > 0");
    }
    if (in.standoff_od_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "standoff_od_mm must be > 0");
    }
    if (in.base_plate_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "base_plate_thickness_mm must be > 0");
    }
    const StandoffSpec* spec = lookupSpec(in.thread_spec);
    if (!spec) {
        r.add("DFM-STANDOFF-SIZE", "error",
              "auto_standoff_floating_point: unknown thread_spec '" +
              in.thread_spec + "' (supported: M2/M2.5/M3/M4/M5/M6)");
        return r;
    }

    if (in.standoff_od_mm < spec->min_od_mm) {
        r.add("DFM-STANDOFF-OD", "error",
              "standoff_od " + std::to_string(in.standoff_od_mm) +
              " mm < min " + std::to_string(spec->min_od_mm) +
              " for thread " + spec->key);
    }

    // Wall thickness check.
    const double wall = (in.standoff_od_mm - spec->pilot_dia_mm) / 2.0;
    if (wall < spec->min_wall_mm) {
        r.add("DFM-STANDOFF-OD", "error",
              "wall thickness " + std::to_string(wall) +
              " mm < min " + std::to_string(spec->min_wall_mm));
    }

    // Thread-engagement length rule: tap depth ≥ 3 × pilot_dia.
    if (in.height_mm < 3.0 * spec->pilot_dia_mm) {
        r.add("DFM-STANDOFF-H", "warning",
              "height " + std::to_string(in.height_mm) +
              " mm < 3 × pilot_dia (" +
              std::to_string(3.0 * spec->pilot_dia_mm) + " mm) — thread engagement low");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "auto_standoff_floating_point DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const StandoffSpec* spec = lookupSpec(in.thread_spec);
    // spec is non-null because validate passed.

    const gp_Dir adir = in.axis_dir;
    const double r_od    = in.standoff_od_mm / 2.0;
    const double r_pilot = spec->pilot_dia_mm / 2.0;

    // ── Context probe: is there a base near Z=0? ─────────────────────────
    const bool hasBase = baseAtZeroPresent(wp,
        in.position_x_mm, in.position_y_mm, /* zTol */ 0.5);

    TopoDS_Shape working = wp.shape();

    // ── Sub-feature 1: drop thin base plate if missing ──────────────────
    double baseLengthSide_mm = 0.0;
    if (!hasBase) {
        // Drop a square base plate of side 2 × standoff_od centred on XY
        // and thickness = base_plate_thickness_mm, sitting with TOP at z=0.
        baseLengthSide_mm = std::max(4.0 * r_od, 6.0);
        const gp_Pnt baseOrigin(
            in.position_x_mm - baseLengthSide_mm / 2.0,
            in.position_y_mm - baseLengthSide_mm / 2.0,
            -in.base_plate_thickness_mm);
        const TopoDS_Shape baseTool = pr::box(
            gp_Ax2(baseOrigin, gp::DZ()),
            baseLengthSide_mm, baseLengthSide_mm, in.base_plate_thickness_mm);
        working = pr::fuse(working, baseTool);
    }

    // ── Sub-feature 2/3: fuse cylindrical standoff outer ───────────────
    //
    // Build column from Z=0 up to z = height_mm.  axis_dir is +Z by
    // convention.  We FUSE the outer column then CUT the pilot — per
    // project rule, for concentric overlapping cylinders we'd fuse both
    // as a single solid, but in this case the inner is a CUT so the
    // order is: fuse column → cut pilot from column.
    const gp_Pnt colBottom(in.position_x_mm, in.position_y_mm, 0.0);
    const gp_Ax2 axCol(colBottom, adir);
    const TopoDS_Shape colOuter = pr::cylinder(axCol, r_od, in.height_mm);
    // To "fuse cylindrical standoff + tapped top" per the spec, we
    // produce a second, slightly larger cylinder at the top to seat the
    // washer, then fuse it onto the column.  Top seat: OD + 0.5 mm,
    // height 0.5 mm.
    const double rSeat = r_od + 0.5;
    const gp_Pnt seatBottom(in.position_x_mm, in.position_y_mm, in.height_mm - 0.5);
    const gp_Ax2 axSeat(seatBottom, adir);
    const TopoDS_Shape topSeat = pr::cylinder(axSeat, rSeat, 0.5);

    // Per project rule: fuse concentric overlapping cylinders into a
    // single connected solid BEFORE booleans.
    const TopoDS_Shape colFused = pr::fuse(colOuter, topSeat);

    // Fuse column-with-seat onto the workpiece.
    TopoDS_Shape withColumn = pr::fuse(working, colFused);

    // ── Sub-feature 4: cut tapped pilot from the top ────────────────────
    const double kOver = 0.1;
    const gp_Pnt pilotTop(
        in.position_x_mm,
        in.position_y_mm,
        in.height_mm + kOver);
    const gp_Ax2 axPilot(pilotTop, gp_Dir(0, 0, -1));
    // pilot depth: 3 × pilot_dia OR full height − 0.5 mm, whichever smaller.
    const double pilotDepth = std::min(in.height_mm - 0.5,
                                       std::max(3.0 * spec->pilot_dia_mm,
                                                in.height_mm * 0.6));
    const TopoDS_Shape pilotTool = pr::cylinder(axPilot, r_pilot,
                                                pilotDepth + kOver);
    const TopoDS_Shape newShape = pr::cut(withColumn, pilotTool);

    // ── Bookkeeping ─────────────────────────────────────────────────────
    const double vBase  = hasBase ? 0.0 :
        baseLengthSide_mm * baseLengthSide_mm * in.base_plate_thickness_mm;
    const double vCol   = M_PI * r_od    * r_od    * in.height_mm;
    const double vSeat  = M_PI * (rSeat * rSeat - r_od * r_od) * 0.5;
    const double vPilot = M_PI * r_pilot * r_pilot * pilotDepth;

    json params = {
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "height_mm",      in.height_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "thread_spec",    in.thread_spec },
        { "standoff_od_mm", in.standoff_od_mm },
        { "base_plate_thickness_mm", in.base_plate_thickness_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    hasBase ? 3 : 4 },
        { "base_synthesized",    !hasBase },
        { "thread_spec",         in.thread_spec },
        { "standoff_od_mm",      in.standoff_od_mm },
        { "pilot_dia_mm",        spec->pilot_dia_mm },
        { "height_mm",           in.height_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "standard",            "ISO 68-1" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = hasBase
        ? "mould;drill;tap"
        : "mould-base;mould;drill;tap";
    tooling.tool_dia_mm       = spec->pilot_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.stock_removed_mm3 = vPilot;
    tooling.est_cycle_time_s  = 60.0;
    tooling.extra = {
        { "base_volume_mm3",    vBase },
        { "column_volume_mm3",  vCol },
        { "seat_volume_mm3",    vSeat },
        { "pilot_volume_mm3",   vPilot },
        { "added_volume_mm3",   vBase + vCol + vSeat },
        { "removed_volume_mm3", vPilot },
        { "net_delta_mm3",      (vBase + vCol + vSeat) - vPilot },
        { "pilot_depth_mm",     pilotDepth },
        { "standard",           "ISO 68-1" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::auto_standoff_floating_point: hasBase={} spec={} OD={}",
                  hasBase, in.thread_spec, in.standoff_od_mm);

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

bool axesNear(const gp_Ax1& a, const gp_Ax1& b, double posTol)
{
    const double dot = std::abs(
        a.Direction().X() * b.Direction().X() +
        a.Direction().Y() * b.Direction().Y() +
        a.Direction().Z() * b.Direction().Z());
    if (dot < 0.999) return false;
    const gp_Pnt& pa = a.Location();
    const gp_Pnt& pb = b.Location();
    return std::sqrt(std::pow(pa.X() - pb.X(), 2) +
                     std::pow(pa.Y() - pb.Y(), 2)) < posTol;
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

    // Pair larger (OD) with smaller (pilot) cylinder on same axis.
    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (!axesNear(cyls[i].axis, cyls[j].axis, 0.5)) continue;
            const CylRec& small = (cyls[i].radius < cyls[j].radius) ? cyls[i] : cyls[j];
            const CylRec& large = (cyls[i].radius < cyls[j].radius) ? cyls[j] : cyls[i];
            if (std::abs(large.radius - small.radius) < 0.5) continue;

            // Match pilot diameter against table.
            const std::string keyGuess = [&]() -> std::string {
                for (const auto& s : kStandoffTable) {
                    if (std::abs(2.0 * small.radius - s.pilot_dia_mm) < 0.3 &&
                        2.0 * large.radius >= s.min_od_mm - 0.5) {
                        return s.key;
                    }
                }
                return {};
            }();
            if (keyGuess.empty()) continue;

            json recovered = {
                { "position_x_mm",  small.axis.Location().X() },
                { "position_y_mm",  small.axis.Location().Y() },
                { "axis_dir",       { 0.0, 0.0, 1.0 } },
                { "thread_spec",    keyGuess },
                { "standoff_od_mm", 2.0 * large.radius },
                { "height_mm",      10.0 },
                { "base_plate_thickness_mm", 1.2 },
            };
            json matched = {
                { "od_face_id",    large.idx },
                { "pilot_face_id", small.idx },
                { "via",           "geometric_primary" },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.78, matched });
        }
    }

    if (out.empty()) emitMetadataReplay();
    return out;
}

}  // namespace koocadcam::skill::auto_standoff_floating_point
