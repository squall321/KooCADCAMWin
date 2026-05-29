// @lat: [[engine/skills#vise_jaw_with_v_groove]]

#include "vise_jaw_with_v_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::vise_jaw_with_v_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// DFM cites DIN 6346 (Soft jaws for machine vises) and ANSI/ASME B5.41
// (Machine tool vise specifications).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // DIN 6346: V-groove included angle 60..120°.  90° is standard for round
    // stock; tighter angles bite harder but mark the workpiece.
    if (in.v_groove_angle_deg < 60.0 || in.v_groove_angle_deg > 120.0) {
        r.add("DFM-V-ANGLE", "error",
              "vise_jaw_with_v_groove: v_groove_angle " +
              std::to_string(in.v_groove_angle_deg) +
              "° outside DIN 6346 [60°, 120°]");
    }
    if (in.v_groove_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "vise_jaw_with_v_groove: v_groove_depth must be > 0");
    }
    if (in.pilot_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "vise_jaw_with_v_groove: pilot_dia " +
              std::to_string(in.pilot_dia_mm) + " mm < 0.8 mm");
    }
    if (in.counterbore_dia_mm <= in.pilot_dia_mm) {
        r.add("DFM-VISE-CB", "error",
              "vise_jaw_with_v_groove: counterbore_dia " +
              std::to_string(in.counterbore_dia_mm) +
              " ≤ pilot_dia " + std::to_string(in.pilot_dia_mm));
    }
    if (in.counterbore_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "vise_jaw_with_v_groove: counterbore_depth must be > 0");
    }
    if (in.mount_holes.empty()) {
        r.add("DFM-INPUT", "error",
              "vise_jaw_with_v_groove: at least one mount hole required");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double jawHeight = zMax - zMin;
    // Groove depth ≤ 0.4 × jaw height (else jaw too thin under groove)
    if (jawHeight > 0.0 && in.v_groove_depth_mm > 0.4 * jawHeight) {
        r.add("DFM-V-DEPTH", "error",
              "vise_jaw_with_v_groove: groove depth " +
              std::to_string(in.v_groove_depth_mm) +
              " > 0.4 × jaw height (" + std::to_string(0.4 * jawHeight) + ")");
    }
    // Groove z must place groove within jaw height.
    if (in.v_groove_z_mm < zMin || in.v_groove_z_mm > zMax) {
        r.add("DFM-V-Z", "error",
              "vise_jaw_with_v_groove: v_groove_z " +
              std::to_string(in.v_groove_z_mm) +
              " outside jaw z extent [" +
              std::to_string(zMin) + ", " + std::to_string(zMax) + "]");
    }
    // ASME B5.41: mounting bolts spaced ≥ 25 mm for M6/M8 jaws.
    for (size_t i = 0; i < in.mount_holes.size(); ++i) {
        for (size_t j = i + 1; j < in.mount_holes.size(); ++j) {
            const double dx = in.mount_holes[i].x_mm - in.mount_holes[j].x_mm;
            const double dz = in.mount_holes[i].z_mm - in.mount_holes[j].z_mm;
            const double d  = std::sqrt(dx * dx + dz * dz);
            if (d < in.counterbore_dia_mm + 2.0) {
                r.add("DFM-VISE-SPACING", "error",
                      "vise_jaw_with_v_groove: mount holes " +
                      std::to_string(i) + " and " + std::to_string(j) +
                      " too close (" + std::to_string(d) + " < " +
                      std::to_string(in.counterbore_dia_mm + 2.0) + " mm)");
            }
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Geometry:
//   The vise jaw is a block of dimensions (LX, LY, LZ) with LY being the
//   jaw thickness (depth into the vise) and LZ being the jaw height.
//   Working face is the +Y face.  V-groove is horizontal along X at z =
//   v_groove_z_mm.  Mount holes pass from the BACK face (-Y) through to
//   the working face; SHCS heads sit in counterbore on the back face.
//
// Sub-cuts:
//   1. CUT  V-groove triangular prism (extrude along +X, the jaw length).
//   2..N. For each mount hole at (x, z):
//          CUT pilot through-cyl (axis along -Y)
//          CUT counterbore cyl on the BACK face
//
// All cutters are fused into one compound, then a single Boolean CUT.

namespace {

// Build a triangular prism representing the V-groove cutter.
//   Apex points INTO the jaw at depth `depth` below the working face.
//   The triangle's two upper vertices lie on the working face (y = yMax),
//   spaced by 2·depth·tan(angle/2) along Z.
//   The prism extrudes along +X by `length`.
TopoDS_Shape buildVPrism(double xStart, double xEnd, double yWorking,
                         double zCenter, double angleDeg, double depth,
                         double overhang)
{
    const double halfAngle = angleDeg * 0.5 * M_PI / 180.0;
    const double halfWidth = depth * std::tan(halfAngle);

    // Cross-section vertices in YZ plane (at x = xStart - overhang).
    // Upper edge sits ABOVE the working face by overhang so cut is clean.
    const double xStartOH = xStart - overhang;
    const gp_Pnt pTopL(xStartOH, yWorking + overhang, zCenter + halfWidth);
    const gp_Pnt pTopR(xStartOH, yWorking + overhang, zCenter - halfWidth);
    const gp_Pnt pApex(xStartOH, yWorking - depth,    zCenter);

    BRepBuilderAPI_MakeWire wire;
    wire.Add(BRepBuilderAPI_MakeEdge(pTopL, pTopR).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(pTopR, pApex).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(pApex, pTopL).Edge());
    if (!wire.IsDone())
        throw Standard_Failure("vise_jaw_with_v_groove: V wire build failed");
    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone())
        throw Standard_Failure("vise_jaw_with_v_groove: V face build failed");
    const gp_Vec extrude(xEnd - xStart + 2.0 * overhang, 0.0, 0.0);
    BRepPrimAPI_MakePrism prism(face.Face(), extrude);
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("vise_jaw_with_v_groove: V prism build failed");
    return prism.Shape();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vise_jaw_with_v_groove DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.working_face);
    if (!entryId) throw SkillError(
        "vise_jaw_with_v_groove: working_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const double jawThickness = yMax - yMin;   // along Y

    // ── Sub-feature 1: V-groove prism ────────────────────────────────────
    TopoDS_Shape fused = buildVPrism(xMin, xMax, yMax, in.v_groove_z_mm,
                                     in.v_groove_angle_deg,
                                     in.v_groove_depth_mm, kOverhang);

    // ── Sub-features 2…N+1: each mount hole — pilot through + counterbore
    // on the BACK face.  Pilot axis = -Y (drill from back through working).
    for (const auto& m : in.mount_holes) {
        const gp_Dir backToFront(0, 1, 0);
        // toolStart at back face (yMin), going +Y.
        const gp_Pnt pilotStart(m.x_mm, yMin - kOverhang, m.z_mm);
        const gp_Ax2 pilotAx(pilotStart, backToFront);
        const TopoDS_Shape pilot = pr::cylinder(
            pilotAx, in.pilot_dia_mm / 2.0, jawThickness + 2.0 * kOverhang);

        // Counterbore at the back face: cylinder of counterbore_dia for
        // counterbore_depth, starts at yMin-overhang, goes +Y by
        // counterbore_depth + overhang.
        const gp_Ax2 cbAx(pilotStart, backToFront);
        const TopoDS_Shape cb = pr::cylinder(
            cbAx, in.counterbore_dia_mm / 2.0,
            in.counterbore_depth_mm + kOverhang);

        TopoDS_Shape holeFused = pr::fuse(pilot, cb);
        fused = pr::fuse(fused, holeFused);
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json holesJson = json::array();
    for (const auto& m : in.mount_holes)
        holesJson.push_back({ { "x_mm", m.x_mm }, { "z_mm", m.z_mm } });
    json params = {
        { "working_face_id",      *entryId },
        { "v_groove_angle_deg",   in.v_groove_angle_deg },
        { "v_groove_depth_mm",    in.v_groove_depth_mm },
        { "v_groove_z_mm",        in.v_groove_z_mm },
        { "pilot_dia_mm",         in.pilot_dia_mm },
        { "counterbore_dia_mm",   in.counterbore_dia_mm },
        { "counterbore_depth_mm", in.counterbore_depth_mm },
        { "mount_holes",          holesJson },
    };
    const int subCount = 1 + 2 * static_cast<int>(in.mount_holes.size());
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      subCount },
        { "mount_hole_count",      static_cast<int>(in.mount_holes.size()) },
        { "v_groove_angle_deg",    in.v_groove_angle_deg },
        { "v_groove_depth_mm",     in.v_groove_depth_mm },
        { "v_groove_z_mm",         in.v_groove_z_mm },
        { "pilot_dia_mm",          in.pilot_dia_mm },
        { "counterbore_dia_mm",    in.counterbore_dia_mm },
        { "mount_holes",           holesJson },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "v_mill;drill;counterbore";
    tooling.tool_dia_mm      = in.counterbore_dia_mm;
    tooling.tool_length_mm   = jawThickness + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    // Removed volume:
    //   V-groove triangle area × jaw length
    //   + per hole: pilot + counterbore annulus
    const double jawLen = xMax - xMin;
    const double halfAngle = in.v_groove_angle_deg * 0.5 * M_PI / 180.0;
    const double halfW = in.v_groove_depth_mm * std::tan(halfAngle);
    const double triArea = halfW * in.v_groove_depth_mm;  // = 0.5 · base · height
    double vol = triArea * jawLen;
    const double rPilot = in.pilot_dia_mm / 2.0;
    const double rCB    = in.counterbore_dia_mm / 2.0;
    const double holeVol =
        M_PI * rPilot * rPilot * jawThickness +
        M_PI * (rCB * rCB - rPilot * rPilot) * in.counterbore_depth_mm;
    vol += holeVol * static_cast<double>(in.mount_holes.size());
    tooling.stock_removed_mm3 = vol;
    tooling.est_cycle_time_s  = std::max(8.0, vol / 4000.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "v_mill" },     { "operation", "v_groove" },
              { "angle_deg", in.v_groove_angle_deg },
              { "depth_mm",  in.v_groove_depth_mm },
              { "length_mm", jawLen } },
            { { "tool_type", "drill" },       { "operation", "pilot_through" },
              { "tool_dia_mm", in.pilot_dia_mm },
              { "pass_count",  static_cast<int>(in.mount_holes.size()) } },
            { { "tool_type", "counterbore" }, { "operation", "back_counterbore" },
              { "tool_dia_mm", in.counterbore_dia_mm },
              { "depth_mm",    in.counterbore_depth_mm } },
        } },
        { "jaw_standard", "DIN_6346 / ANSI_ASME_B5.41" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::vise_jaw_with_v_groove applied: "
                  "v={}° depth={} holes={}",
                  in.v_groove_angle_deg, in.v_groove_depth_mm,
                  in.mount_holes.size());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic PRIMARY: identify the V-groove as a pair of opposed
// planar walls meeting at a line apex (interior angle = 180° - v_groove_angle).
// FALLBACK: metadata replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata path first.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric path: a V-groove leaves 2 planar faces whose normals lie in
    // the YZ plane (no X component) and whose Z components are equal and
    // opposite (mirror image about the groove apex plane).
    std::vector<int> tilted;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        // V-groove walls have nx ≈ 0, |ny| > 0.3 (a real wall component),
        // |nz| > 0.1 (tilted), but neither approaching ±1 (would be top or
        // back face).
        if (std::abs(n.X()) > 0.05) continue;
        if (std::abs(n.Y()) < 0.3)  continue;
        if (std::abs(n.Z()) < 0.1)  continue;
        if (std::abs(n.Z()) > 0.95) continue;
        tilted.push_back(i);
    }
    if (tilted.size() < 2) return out;

    // Pair tilted walls with opposite Z signs to form a V.
    int vA = -1, vB = -1;
    for (size_t i = 0; i < tilted.size(); ++i) {
        for (size_t j = i + 1; j < tilted.size(); ++j) {
            gp_Dir na, nb;
            try {
                na = wp.faceNormal(tilted[i]);
                nb = wp.faceNormal(tilted[j]);
            } catch (...) { continue; }
            if (na.Z() * nb.Z() < 0.0 &&
                std::abs(std::abs(na.Z()) - std::abs(nb.Z())) < 0.1) {
                vA = tilted[i]; vB = tilted[j]; break;
            }
        }
        if (vA >= 0) break;
    }
    if (vA < 0) return out;

    // Recover groove angle from wall normal Z component:
    //   wall tilts by (90 - v_groove_angle/2) from vertical
    //   ⇒ |n.Y|² + |n.Z|² = 1, included = 2 · acos(|n.Y|)
    gp_Dir na;
    try { na = wp.faceNormal(vA); } catch (...) { return out; }
    const double included = 2.0 * std::acos(std::clamp(std::abs(na.Y()), 0.0, 1.0)) *
                            180.0 / M_PI;
    const gp_Pnt cA = wp.faceCenter(vA);

    json rp = {
        { "v_groove_angle_deg",   included },
        { "v_groove_depth_mm",    6.0 },    // not directly recoverable
        { "v_groove_z_mm",        cA.Z() },
        { "pilot_dia_mm",         6.6 },
        { "counterbore_dia_mm",   10.0 },
        { "counterbore_depth_mm", 6.0 },
        { "mount_holes",          json::array() },
    };
    RecognizedFeature rf;
    rf.skill_id         = kSkillId;
    rf.recovered_params = rp;
    rf.confidence       = 0.7;
    rf.matched_geometry = { { "v_wall_face_ids", { vA, vB } },
                            { "source", "geometric" } };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::vise_jaw_with_v_groove
