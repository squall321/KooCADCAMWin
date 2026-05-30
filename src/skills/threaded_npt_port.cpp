// @lat: [[engine/skills#threaded_npt_port]]

#include "threaded_npt_port.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace koocadcam::skill::threaded_npt_port {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// NPT half-angle (1.7899° / side; full taper is 1°47'/foot ≈ 1.79°/side).
constexpr double kNptHalfAngleDeg = 1.7899;

namespace {

struct NptEntry {
    const char* size;
    double      nominal_od_mm;
    int         threads_per_inch;
    double      l1_depth_mm;     // ASME B1.20.1 effective engagement length
};

constexpr std::array<NptEntry, 6> kNptTable {{
    { "1/8",  10.29, 27,    4.10 },
    { "1/4",  13.72, 18,    5.79 },
    { "3/8",  17.15, 18,    6.10 },
    { "1/2",  21.34, 14,    8.13 },
    { "3/4",  26.67, 14,    8.61 },
    { "1",    33.40, 12,   10.16 },  // 11.5 rounded to 12 for slice-1
}};

const NptEntry* find(const std::string& s)
{
    for (const auto& e : kNptTable) if (s == e.size) return &e;
    return nullptr;
}

}  // namespace

bool resolveNptSize(const std::string& size,
                    double& nominal_od_mm,
                    int&    threads_per_inch,
                    double& l1_depth_mm)
{
    const NptEntry* e = find(size);
    if (!e) return false;
    nominal_od_mm    = e->nominal_od_mm;
    threads_per_inch = e->threads_per_inch;
    l1_depth_mm      = e->l1_depth_mm;
    return true;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.recess_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "threaded_npt_port: recess_depth_mm must be > 0");
    }
    if (in.chamfer_leg_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "threaded_npt_port: chamfer_leg_mm must be >= 0");
    }
    if (in.npt_size.empty()) {
        r.add("DFM-NPT-SIZE", "error", "threaded_npt_port: npt_size is empty");
        return r;
    }
    const NptEntry* e = find(in.npt_size);
    if (!e) {
        r.add("DFM-NPT-SIZE", "error",
              "threaded_npt_port: unknown npt_size '" + in.npt_size +
              "' (supported: 1/8, 1/4, 3/8, 1/2, 3/4, 1)");
        return r;
    }

    // Pitch in mm = 25.4 / TPI.
    const double pitch_mm = 25.4 / static_cast<double>(e->threads_per_inch);
    const double totalDepth = e->l1_depth_mm + in.recess_depth_mm;

    if (totalDepth > in.part_thickness_mm) {
        r.add("DFM-NPT-DEPTH", "error",
              "threaded_npt_port: required depth (L1 + recess) " +
              std::to_string(totalDepth) +
              " mm > part_thickness " + std::to_string(in.part_thickness_mm) + " mm");
    }

    // Per ASME B1.20.1, minimum wall around an NPT port ≈ 2× pitch.
    // We approximate "wall" as half the part thickness MINUS the OD/2.
    // (Caller's responsibility to back this with a real distance check.)
    // We simply WARN if the part thickness margin is < 2× pitch.
    const double radialMargin = std::max(0.0,
        (in.part_thickness_mm - e->nominal_od_mm) / 2.0);
    if (radialMargin < 2.0 * pitch_mm) {
        r.add("DFM-NPT-WALL", "warning",
              "threaded_npt_port: estimated wall around port " +
              std::to_string(radialMargin) + " mm < 2× pitch " +
              std::to_string(2.0 * pitch_mm) + " mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "threaded_npt_port DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const NptEntry* e = find(in.npt_size);
    if (!e) throw SkillError("threaded_npt_port: npt_size lookup failed unexpectedly");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("threaded_npt_port: face_id datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    constexpr double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Common entry start (matches drill_hole pattern: just outside the
    // workpiece along -adir, for clean Boolean entry).
    gp_Pnt entryStart(
        in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            entryStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        } else {
            entryStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
        }
    }

    const double nomR = e->nominal_od_mm / 2.0;
    const double l1   = e->l1_depth_mm;
    const double taperPerSide = std::tan(kNptHalfAngleDeg * M_PI / 180.0);
    // Small end radius (deep end of cone).
    const double smallR = std::max(0.1, nomR - taperPerSide * l1);

    // ── Sub-cut #1: NPT tapered hole (cone frustum) ─────────────────────
    //
    // Convention: gp_Ax2(P, V, Vx): V = LOCAL Z (axial).  We position the
    // cone so that its BIG end coincides with the face (entryStart) and
    // its SMALL end is at depth = l1 along adir.  BRepPrimAPI_MakeCone
    // with (axis, r1, r2, height) builds the frustum where r1 sits at
    // axis.Location() and r2 at height along axis.Direction().  So r1 =
    // nomR (face) and r2 = smallR (deep end).
    const gp_Ax2 coneAx(entryStart, adir);
    const TopoDS_Shape nptCone =
        pr::coneFrustum(coneAx, nomR, smallR, l1 + kOverhang);

    // ── Sub-cut #2: counterbore recess at the mouth ─────────────────────
    // Slightly larger than nomR (default +0.5 mm).  Provides tool relief.
    const double recessR = nomR + 0.5;
    const gp_Ax2 recessAx(entryStart, adir);
    const TopoDS_Shape recess = pr::cylinder(
        recessAx, recessR, in.recess_depth_mm + kOverhang);

    // ── Sub-cut #3: face chamfer (45° bevel into the material) ──────────
    // The chamfer removes the sharp edge at the cylinder mouth.  Axis
    // points INTO the material (adir).  At axis.Location (face plane) the
    // cone is wide (recessR + leg); at depth `leg` it narrows to recessR,
    // matching the cylindrical recess wall.  This produces a 45° bevel.
    TopoDS_Shape chamfer;
    if (in.chamfer_leg_mm > 0.0) {
        const gp_Pnt chamfOrigin(
            in.position_x_mm,
            in.position_y_mm,
            entryStart.Z());
        const gp_Ax2 chamfAx(chamfOrigin, adir);
        chamfer = pr::coneFrustum(
            chamfAx,
            recessR + in.chamfer_leg_mm,    // wide at the face
            recessR,                        // narrows to match recess wall
            in.chamfer_leg_mm);
    }

    // Pre-fuse the three concentric cutters into one (NPT cone + recess +
    // chamfer), then a single cut on the workpiece.
    TopoDS_Shape unified = pr::fuse(nptCone, recess);
    if (!chamfer.IsNull()) unified = pr::fuse(unified, chamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    // ── Signature ───────────────────────────────────────────────────────
    const double pitch_mm = 25.4 / static_cast<double>(e->threads_per_inch);

    json params = {
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "npt_size",         in.npt_size },
        { "recess_depth_mm",  in.recess_depth_mm },
        { "chamfer_leg_mm",   in.chamfer_leg_mm },
        { "part_thickness_mm",in.part_thickness_mm },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "subfeature_count",   in.chamfer_leg_mm > 0.0 ? 3 : 2 },
        { "npt_size",           in.npt_size },
        { "nominal_od_mm",      e->nominal_od_mm },           // derived
        { "threads_per_inch",   e->threads_per_inch },         // derived
        { "pitch_mm",           pitch_mm },                    // derived
        { "l1_depth_mm",        l1 },                          // derived
        { "small_end_dia_mm",   2.0 * smallR },                // derived
        { "taper_per_side_deg", kNptHalfAngleDeg },            // standard
        { "recess_depth_mm",    in.recess_depth_mm },
        { "recess_dia_mm",      2.0 * recessR },               // derived
        { "chamfer_leg_mm",     in.chamfer_leg_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "tapered_tap;counterbore;chamfer_mill";
    tooling.tool_dia_mm       = e->nominal_od_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 80.0;             // NPT taps run slow
    tooling.feed_per_tooth_mm = 0.0;              // tap = synchronized feed
    tooling.est_cycle_time_s  = std::max(8.0, l1 * 1.5);
    const double coneVol = M_PI * l1 / 3.0
                         * (nomR * nomR + nomR * smallR + smallR * smallR);
    const double recessVol = M_PI * recessR * recessR * in.recess_depth_mm
                           - M_PI * nomR * nomR * in.recess_depth_mm;
    tooling.stock_removed_mm3 = coneVol + std::max(0.0, recessVol);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "tapered_tap" },
              { "for", "NPT " + in.npt_size },
              { "tpi", e->threads_per_inch },
              { "l1_depth_mm", l1 },
              { "taper_deg_per_side", kNptHalfAngleDeg } },
            { { "tool_type", "counterbore" },
              { "tool_dia_mm", 2.0 * recessR },
              { "depth_mm", in.recess_depth_mm } },
            { { "tool_type", "chamfer_mill" },
              { "leg_mm", in.chamfer_leg_mm },
              { "angle_deg", 45.0 } },
        } },
        { "note", "tapered NPT thread modelled as cone frustum (slice-1)" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::threaded_npt_port applied: {} OD={} L1={}",
                  in.npt_size, e->nominal_od_mm, l1);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay + geometric fallback) ──────────────────
//
// Geometric signature of an NPT port:
//   - 1 conical face (the NPT taper, half-angle ≈ 1.79°/side).
//   - 1 cylindrical face concentric with the cone (the counterbore recess).
//   - Optional 1 conical face at the mouth (45° chamfer, much steeper).
// All coaxial along the port axis.

namespace {

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    if (wp.shape().IsNull()) return out;

    // Scan for cone faces; classify by half-angle (NPT vs. chamfer).
    int nptConeId = -1;
    int chamferId = -1;
    gp_Dir nptAxis(0, 0, 1);
    gp_Pnt nptAxisLoc(0, 0, 0);

    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface surf(wp.face(i));
        if (surf.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cone = surf.Cone();
        const double halfAngDeg = std::abs(cone.SemiAngle()) * 180.0 / M_PI;
        if (halfAngDeg < 5.0 && halfAngDeg > 0.5) {
            // NPT-like cone (≈ 1.79°/side)
            nptConeId  = i;
            nptAxis    = cone.Axis().Direction();
            nptAxisLoc = cone.Axis().Location();
        } else if (halfAngDeg > 30.0 && halfAngDeg < 60.0) {
            // 45° face chamfer
            chamferId = i;
        }
    }
    if (nptConeId < 0) return out;
    (void)nptAxisLoc;  // reserved for future axis-based recovery

    // Look for a cylindrical face co-axial with the NPT cone.
    int recessId = -1;
    double recessR = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        const gp_Dir cyAx = cy.Axis().Direction();
        if (std::abs(std::abs(cyAx.Dot(nptAxis)) - 1.0) > 1e-2) continue;
        recessId = i;
        recessR  = cy.Radius();
        break;
    }
    if (recessId < 0) return out;

    // Determine NPT size by matching nominal OD ≈ 2 × recessR − 1.0 mm
    // (synthesis: recessR = nomR + 0.5).
    const double nomOD = 2.0 * (recessR - 0.5);
    const char* bestSize = "1/4";
    double bestDiff = 1e9;
    constexpr std::array<std::pair<const char*, double>, 6> kSizes {{
        { "1/8", 10.29 }, { "1/4", 13.72 }, { "3/8", 17.15 },
        { "1/2", 21.34 }, { "3/4", 26.67 }, { "1",   33.40 },
    }};
    for (const auto& s : kSizes) {
        const double d = std::abs(nomOD - s.second);
        if (d < bestDiff) { bestDiff = d; bestSize = s.first; }
    }

    // Recover position from cone apex (approx. — apex is where cone narrows
    // to zero, while NPT cone is truncated; entry point is along axis from
    // apex by some distance.  We use recess axis location as the entry.
    BRepAdaptor_Surface recessSurf(wp.face(recessId));
    const gp_Cylinder recessCyl = recessSurf.Cylinder();
    const gp_Pnt recessLoc = recessCyl.Axis().Location();

    json recovered = {
        { "position_x_mm",      recessLoc.X() },
        { "position_y_mm",      recessLoc.Y() },
        { "axis_dir",           { -nptAxis.X(), -nptAxis.Y(), -nptAxis.Z() } },
        { "npt_size",           std::string(bestSize) },
        { "recess_depth_mm",    1.5 },
        { "chamfer_leg_mm",     (chamferId >= 0) ? 0.5 : 0.0 },
        { "part_thickness_mm",  20.0 },
    };
    json matched = {
        { "source",           "geometric_fallback" },
        { "npt_cone_face_id", nptConeId },
        { "recess_face_id",   recessId },
        { "chamfer_face_id",  chamferId },
        { "matched_size",     std::string(bestSize) },
        { "diff_mm",          bestDiff },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
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
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::threaded_npt_port
