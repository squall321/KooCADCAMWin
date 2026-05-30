// @lat: [[engine/skills#banana_socket_compound]]

#include "banana_socket_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::banana_socket_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// References: DIN 42220 (banana plugs/sockets), IEC 61010-031 (touch-safe
// laboratory test leads).  Standard 4 mm banana is OD 4.0 mm ± 0.1 mm.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bore_dia_mm < 3.8 || in.bore_dia_mm > 4.2) {
        r.add("DFM-DIN42220", "error",
              "banana_socket bore_dia " + std::to_string(in.bore_dia_mm) +
              " mm outside DIN 42220 range [3.8, 4.2]");
    }
    if (in.bore_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_depth_mm must be > 0");
    }
    if (in.slit_width_mm < 0.4) {
        r.add("DFM-002", "error",
              "banana_socket slit_width " + std::to_string(in.slit_width_mm) +
              " mm < min 0.4 mm (sub-laser-cuttable)");
    }
    if (in.slit_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "slit_depth_mm must be > 0");
    }
    if (in.slit_depth_mm >= in.bore_depth_mm) {
        r.add("DFM-BANANA-SLIT", "error",
              "slit_depth (" + std::to_string(in.slit_depth_mm) +
              ") must be < bore_depth (" + std::to_string(in.bore_depth_mm) +
              ") — slits cannot exit out of the bottom");
    }
    if (in.retention_groove_depth_mm < 0.2) {
        r.add("DFM-BANANA-CLICK", "error",
              "retention_groove_depth " +
              std::to_string(in.retention_groove_depth_mm) +
              " mm < 0.2 mm — no tactile click");
    }
    if (in.retention_groove_dia_mm <= in.bore_dia_mm) {
        r.add("DFM-BANANA-GROOVE", "error",
              "retention_groove_dia must be > bore_dia");
    }
    if (in.retention_groove_pos_mm <= in.insulation_step_depth_mm ||
        in.retention_groove_pos_mm >= in.bore_depth_mm) {
        r.add("DFM-BANANA-GROOVE", "error",
              "retention_groove_pos must be inside (step_depth, bore_depth)");
    }
    if (in.insulation_step_depth_mm < 1.0) {
        r.add("DFM-IEC61010", "error",
              "insulation_step_depth " +
              std::to_string(in.insulation_step_depth_mm) +
              " mm < 1.0 mm — fails IEC 61010-031 touch-safe shroud");
    }
    if (in.insulation_step_dia_mm <= in.bore_dia_mm) {
        r.add("DFM-BANANA-STEP", "error",
              "insulation_step_dia must be > bore_dia");
    }
    if (in.bore_depth_mm <= in.insulation_step_depth_mm) {
        r.add("DFM-BANANA-GEOM", "error",
              "bore_depth must be > insulation_step_depth");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Chain of cuts (input volume - final volume = expected removed):
//   1. Bore     : π·(d/2)²·bore_depth
//   2. 4 slits  : 4 × slit_width × (groove_dia/2 - bore/2 + extra) × slit_depth
//                 (we model slits as 4 thin rectangular boxes that punch
//                  through the bore wall outward to the slit's exit OD)
//   3. Groove   : π·((groove_dia/2)² - (bore/2)²)·groove_depth
//   4. Step     : π·((step_dia/2)² - (bore/2)²)·step_depth

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "banana_socket_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("banana_socket: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    const gp_Dir adir = in.axis_dir;
    // Only support -Z entry in this slice.
    if (std::abs(adir.X()) > 1e-6 || std::abs(adir.Y()) > 1e-6 || adir.Z() >= 0.0) {
        throw SkillError("banana_socket: only axis_dir = -Z supported");
    }

    const double zEntry = zMax;
    const gp_Pnt entryCenter(in.position_x_mm, in.position_y_mm, zEntry + kOverhang);
    const gp_Ax2 toolAx(entryCenter, adir);

    // 1) Main bore.
    const double rBore = in.bore_dia_mm / 2.0;
    const TopoDS_Shape bore = pr::cylinder(
        toolAx, rBore, in.bore_depth_mm + kOverhang);

    // 2) Insulation step (wider counterbore at entry).
    const double rStep = in.insulation_step_dia_mm / 2.0;
    const TopoDS_Shape stepCyl = pr::cylinder(
        toolAx, rStep, in.insulation_step_depth_mm + kOverhang);

    // 3) Retention groove (annular ring inside the bore wall, axial span =
    //    groove_depth at axial position groove_pos from entry).
    const double rGroove = in.retention_groove_dia_mm / 2.0;
    const gp_Pnt grooveBase(
        in.position_x_mm, in.position_y_mm,
        zEntry - in.retention_groove_pos_mm);
    // groove cylinder extends DOWN by groove_depth (axis = -Z; gp_Ax2 V is
    // local Z; we want the cylinder to extrude along -Z).
    const gp_Ax2 grooveAx(grooveBase, adir);
    const TopoDS_Shape grooveOuter = pr::cylinder(
        grooveAx, rGroove, in.retention_groove_depth_mm);

    // 4) Four axial slits — thin rectangular prisms that punch through
    //    the bore wall outward.  Each slit is a box centered on the bore
    //    axis, oriented along one of the 4 cardinal XY directions, of
    //    extent (rGroove + 0.2) outward × slit_width tangential × slit_depth axial.
    // The slits start at the entry face and extend down by slit_depth.
    const double slitOuterR = rGroove + 0.3;       // punch slightly past groove OD
    const double slitInnerR = 0.0;                  // cross the bore center
    const double slitFullLen = slitOuterR * 2.0;    // diameter span
    const double slitW       = in.slit_width_mm;
    const double slitTopZ    = zEntry + kOverhang;
    const double slitBotZ    = zEntry - in.slit_depth_mm;
    const double slitH       = slitTopZ - slitBotZ;
    (void)slitInnerR;

    // Build the 4 slit boxes oriented at 0°, 45°, 90°, 135° — but the
    // task spec says 4 axial slits equispaced 90° apart; we use 0/45/90/135
    // would be 8.  4 fingers means slits at 0°, 90°, 180°, 270° — two
    // diameters (X and Y), and two crossing slits punch ALL FOUR fingers.
    // We thus build 2 crossed slit boxes (axes along X and Y), each one
    // creating 2 fingers; together they create 4 fingers.
    std::vector<TopoDS_Shape> slits;
    {
        // Slit aligned with world X (long-axis = X):
        // gp_Ax2(origin, V=mainZ, Vx=localX).  Box DX = slitFullLen along Vx,
        // DY tangential = slitW (cross product yields +Y direction since
        // mainZ=+Z, Vx=+X), DZ along V = slitH.
        const gp_Pnt origX(
            in.position_x_mm - slitFullLen / 2.0,
            in.position_y_mm - slitW / 2.0,
            slitBotZ);
        const gp_Ax2 axX(origX, gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
        slits.push_back(pr::box(axX, slitFullLen, slitW, slitH));

        const gp_Pnt origY(
            in.position_x_mm - slitW / 2.0,
            in.position_y_mm - slitFullLen / 2.0,
            slitBotZ);
        const gp_Ax2 axY(origY, gp_Dir(0, 0, 1), gp_Dir(0, 1, 0));
        slits.push_back(pr::box(axY, slitW, slitFullLen, slitH));
    }

    // ── Fuse all overlapping cutters into a single cutter, then ONE cut.
    TopoDS_Shape cutter = pr::fuse(bore, stepCyl);
    cutter = pr::fuse(cutter, grooveOuter);
    for (const auto& s : slits) cutter = pr::fuse(cutter, s);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // ── Signature ──
    json params = {
        { "entry_face_id",             *entryId },
        { "position_x_mm",             in.position_x_mm },
        { "position_y_mm",             in.position_y_mm },
        { "axis_dir",                  { adir.X(), adir.Y(), adir.Z() } },
        { "bore_dia_mm",               in.bore_dia_mm },
        { "bore_depth_mm",             in.bore_depth_mm },
        { "slit_width_mm",             in.slit_width_mm },
        { "slit_depth_mm",             in.slit_depth_mm },
        { "retention_groove_dia_mm",   in.retention_groove_dia_mm },
        { "retention_groove_depth_mm", in.retention_groove_depth_mm },
        { "retention_groove_pos_mm",   in.retention_groove_pos_mm },
        { "insulation_step_dia_mm",    in.insulation_step_dia_mm },
        { "insulation_step_depth_mm",  in.insulation_step_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },                // bore + slits + groove + step
        { "bore_dia_mm",         in.bore_dia_mm },
        { "step_dia_mm",         in.insulation_step_dia_mm },
        { "groove_dia_mm",       in.retention_groove_dia_mm },
        { "slit_count",          4 },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;endmill;groove_tool;slitting_saw";
    tooling.tool_dia_mm       = in.bore_dia_mm;
    tooling.tool_length_mm    = in.bore_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.03;
    const double rB = rBore, rS = rStep, rG = rGroove;
    tooling.stock_removed_mm3 =
        M_PI * rB * rB * in.bore_depth_mm                             // main bore
      + M_PI * (rS*rS - rB*rB) * in.insulation_step_depth_mm          // step annulus
      + M_PI * (rG*rG - rB*rB) * in.retention_groove_depth_mm         // groove annulus
      + 2.0 * slitFullLen * in.slit_width_mm * in.slit_depth_mm * 0.5;// slits (approx, ignoring overlap with bore)
    tooling.est_cycle_time_s = std::max(3.0, in.bore_depth_mm / 20.0 + 4.0);
    tooling.extra = {
        { "standard",         "DIN 42220 / IEC 61010-031" },
        { "operation_note",   "drill bore, counterbore step, plunge groove, slot 4 spring-finger slits" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::banana_socket_compound applied: bore {}x{} faces {}->{}",
                  in.bore_dia_mm, in.bore_depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic (PRIMARY): look for a workpiece with at least 3
// coaxial cylindrical faces of clearly different radii (bore + step + groove),
// arranged in the banana-socket signature.  Confidence 0.7.
// Metadata replay (FALLBACK): inspect FeatureSignatures for kSkillId.

namespace {

struct CylInfo
{
    int    faceIdx = -1;
    double radius  = 0.0;
    gp_Ax1 axis;
    gp_Pnt center;
};

std::vector<CylInfo> collectCylinders(const Workpiece& wp)
{
    std::vector<CylInfo> out;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface s(wp.face(fIdx));
        const gp_Cylinder c = s.Cylinder();
        CylInfo info;
        info.faceIdx = fIdx;
        info.radius  = c.Radius();
        info.axis    = c.Axis();
        info.center  = wp.faceCenter(fIdx);
        out.push_back(info);
    }
    return out;
}

bool axesParallel(const gp_Ax1& a, const gp_Ax1& b, double tolDeg = 1.0)
{
    const gp_Dir da = a.Direction();
    const gp_Dir db = b.Direction();
    const double dot = std::abs(da.X()*db.X() + da.Y()*db.Y() + da.Z()*db.Z());
    return dot >= std::cos(tolDeg * M_PI / 180.0);
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay first (highest-confidence path).
    for (const auto& sig : wp.features()) {
        if (sig.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = sig.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }

    // Geometric heuristic: ≥3 COAXIAL cylinders of distinct radii AROUND
    // a vertical bore axis, PLUS planar slit walls that share the bore
    // axis line.  Axes must AGREE in 3D direction (parallel) AND share a
    // common axis-line location to be truly coaxial — not just parallel.
    const auto cyls = collectCylinders(wp);
    if (cyls.size() < 3) return out;

    // Co-axial means: directions are parallel AND axis lines pass through
    // the same XY position (within tol).  Strictly stronger than `parallel`.
    auto coaxial = [](const gp_Ax1& a, const gp_Ax1& b, double tolMm = 0.5) {
        const gp_Dir da = a.Direction();
        const gp_Dir db = b.Direction();
        if (std::abs(std::abs(da.X()*db.X()+da.Y()*db.Y()+da.Z()*db.Z()) - 1.0)
            > 1e-2) return false;
        // Project A.Location onto B's line; distance should be ~0.
        const gp_Vec v(b.Location(), a.Location());
        const gp_Vec d(b.Direction());
        const gp_Vec proj = d * v.Dot(d);
        return (v - proj).Magnitude() < tolMm;
    };

    std::vector<std::vector<size_t>> groups;
    std::vector<bool> used(cyls.size(), false);
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (used[i]) continue;
        std::vector<size_t> g{ i };
        used[i] = true;
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (used[j]) continue;
            if (coaxial(cyls[i].axis, cyls[j].axis)) {
                g.push_back(j);
                used[j] = true;
            }
        }
        if (g.size() >= 3) groups.push_back(g);
    }

    // Count slit walls: planar faces whose center is close to the bore
    // axis and whose normal is purely horizontal (perpendicular to axis).
    auto countSlitWalls = [&](const gp_Ax1& axis) {
        int n = 0;
        for (int i = 0; i < wp.faceCount(); ++i) {
            if (!wp.isFacePlanar(i)) continue;
            gp_Dir nrm;
            try { nrm = wp.faceNormal(i); } catch (...) { continue; }
            // Normal perpendicular to bore axis (here: |n.Z|≈0).
            const gp_Dir ad = axis.Direction();
            const double dotZ = std::abs(nrm.X()*ad.X()+nrm.Y()*ad.Y()+nrm.Z()*ad.Z());
            if (dotZ > 5e-2) continue;
            const gp_Pnt fc = wp.faceCenter(i);
            const gp_Vec v(axis.Location(), fc);
            const gp_Vec d(ad);
            const gp_Vec proj = d * v.Dot(d);
            if ((v - proj).Magnitude() < 5.0) ++n;
        }
        return n;
    };

    for (const auto& g : groups) {
        // Distinct radii required.
        std::vector<double> radii;
        for (size_t idx : g) radii.push_back(cyls[idx].radius);
        std::sort(radii.begin(), radii.end());
        int distinct = 1;
        for (size_t i = 1; i < radii.size(); ++i) {
            if (radii[i] - radii[i - 1] > 0.1) ++distinct;
        }
        if (distinct < 3) continue;

        // Banana-socket marker: smallest radius ~2 mm (4 mm bore).
        if (radii.front() < 1.7 || radii.front() > 2.2) continue;

        const int slitWalls = countSlitWalls(cyls[g.front()].axis);
        // 4 slits give 8 planar walls.  Be lenient.
        const bool slitsDetected = (slitWalls >= 4);

        json recovered = {
            { "position_x_mm",            cyls[g.front()].axis.Location().X() },
            { "position_y_mm",            cyls[g.front()].axis.Location().Y() },
            { "axis_dir", { cyls[g.front()].axis.Direction().X(),
                            cyls[g.front()].axis.Direction().Y(),
                            cyls[g.front()].axis.Direction().Z() } },
            { "bore_dia_mm",              2.0 * radii.front() },
            { "insulation_step_dia_mm",   2.0 * radii.back() },
            { "retention_groove_dia_mm",  2.0 * radii[radii.size() / 2] },
        };
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = recovered;
        r.confidence       = slitsDetected ? 0.85 : 0.70;
        r.matched_geometry = {
            { "coaxial_cyl_count", static_cast<int>(g.size()) },
            { "distinct_radii",    distinct },
            { "slit_wall_count",   slitWalls },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::banana_socket_compound
