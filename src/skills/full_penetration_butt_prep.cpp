// @lat: [[engine/skills#full_penetration_butt_prep]]

#include "full_penetration_butt_prep.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace koocadcam::skill::full_penetration_butt_prep {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.plate_t <= 0.0) {
        r.add("DFM-INPUT", "error",
              "full_penetration_butt_prep: plate_t must be > 0");
    }
    if (in.plate_t > 0.0 && in.plate_t < 3.0) {
        r.add("DFM-WELD-PLATE", "error",
              "full_penetration_butt_prep: plate_t " +
              std::to_string(in.plate_t) +
              " mm < 3 mm — no prep needed (AWS BTC-P4)");
    }
    if (in.groove_angle < 30.0 || in.groove_angle > 90.0) {
        r.add("DFM-WELD-ANGLE", "error",
              "full_penetration_butt_prep: groove_angle " +
              std::to_string(in.groove_angle) +
              "° outside [30°, 90°] (single-V AWS A2.4)");
    }
    if (in.root_face_mm < 0.5) {
        r.add("DFM-WELD-LAND", "error",
              "full_penetration_butt_prep: root_face_mm " +
              std::to_string(in.root_face_mm) +
              " mm < 0.5 mm minimum (DIN EN ISO 9692-1)");
    }
    if (in.plate_t > 0.0 && in.root_face_mm > in.plate_t / 3.0) {
        r.add("DFM-WELD-LAND", "error",
              "full_penetration_butt_prep: root_face_mm " +
              std::to_string(in.root_face_mm) +
              " mm > plate_t/3 = " +
              std::to_string(in.plate_t / 3.0));
    }
    if (in.edge_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "full_penetration_butt_prep: edge_length_mm must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "full_penetration_butt_prep DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId)
        throw SkillError("full_penetration_butt_prep: entry_face unresolved");

    // Derived geometry — groove vertical depth = plate_t − root_face,
    // half-angle from the vertical.
    const double grooveDepth  = in.plate_t - in.root_face_mm;
    const double halfAngleRad = (in.groove_angle * 0.5) * M_PI / 180.0;
    const double halfWidthTop = grooveDepth * std::tan(halfAngleRad);

    // Plate bbox — we orient the groove so it sits at (edge_center_x,
    // edge_center_y) running along edge_dir on the top face (entry face
    // assumed +Z, simplification consistent with mill_keyway / slot_weld).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    // Local frame: xLoc = edge_dir projected to XY plane.
    const gp_Vec edgeVec(in.edge_dir.X(), in.edge_dir.Y(), 0.0);
    if (edgeVec.Magnitude() < 1e-6)
        throw SkillError("full_penetration_butt_prep: edge_dir is vertical");
    const gp_Dir xLoc(edgeVec.X(), edgeVec.Y(), 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);  // 90° CCW

    const double L         = in.edge_length_mm;
    const double overhang  = 0.05;
    const double cutLen    = L + 2.0 * overhang;

    // ── Sub-feature 1: bevel A (the +y side of the V) ─────────────────
    //
    // Strategy: build an axis-aligned box that covers the +y half of the
    // groove footprint, then ROTATE it about the bottom edge of the V by
    // halfAngle so it cuts at the slanted groove wall.
    const gp_Pnt boxAOrigin(
        in.edge_center_x_mm - xLoc.X() * L / 2.0 - xLoc.X() * overhang,
        in.edge_center_y_mm - xLoc.Y() * L / 2.0 - xLoc.Y() * overhang,
        topZ - grooveDepth);
    // gp_Ax2(P, V=+Z, Vx=xLoc): DX = cutLen, DY = halfWidthTop+small,
    // DZ = grooveDepth + overhang.
    const gp_Ax2 axA(boxAOrigin, gp::DZ(), xLoc);
    const double bevelY = halfWidthTop + 0.01;
    const TopoDS_Shape boxA = pr::box(axA, cutLen, bevelY, grooveDepth + overhang);

    // Rotate about the bottom edge (z = topZ - grooveDepth, y = edge_center_y)
    // so the slope faces in toward the center.
    gp_Trsf rotA;
    gp_Ax1 hingeA(
        gp_Pnt(in.edge_center_x_mm - xLoc.X() * L / 2.0,
               in.edge_center_y_mm,
               topZ - grooveDepth),
        xLoc);
    rotA.SetRotation(hingeA, halfAngleRad);
    BRepBuilderAPI_Transform tA(boxA, rotA, true);
    const TopoDS_Shape bevelACutter = tA.Shape();

    // ── Sub-feature 2: bevel B (the -y side, mirror) ──────────────────
    const gp_Pnt boxBOrigin(
        in.edge_center_x_mm - xLoc.X() * L / 2.0 - xLoc.X() * overhang
            - yLoc.X() * bevelY,
        in.edge_center_y_mm - xLoc.Y() * L / 2.0 - xLoc.Y() * overhang
            - yLoc.Y() * bevelY,
        topZ - grooveDepth);
    const gp_Ax2 axB(boxBOrigin, gp::DZ(), xLoc);
    const TopoDS_Shape boxB = pr::box(axB, cutLen, bevelY, grooveDepth + overhang);
    gp_Trsf rotB;
    gp_Ax1 hingeB(
        gp_Pnt(in.edge_center_x_mm - xLoc.X() * L / 2.0,
               in.edge_center_y_mm,
               topZ - grooveDepth),
        xLoc);
    rotB.SetRotation(hingeB, -halfAngleRad);
    BRepBuilderAPI_Transform tB(boxB, rotB, true);
    const TopoDS_Shape bevelBCutter = tB.Shape();

    // ── Sub-feature 3: root radius cylinder along bottom of V ─────────
    //
    // A small cylinder sweeping along xLoc at z = topZ - grooveDepth,
    // y = edge_center_y removes the sharp interior corner.
    constexpr double kRootRadius = 0.5;   // mm — typical V-groove root R
    const gp_Pnt cylStart(
        in.edge_center_x_mm - xLoc.X() * L / 2.0 - xLoc.X() * overhang,
        in.edge_center_y_mm - xLoc.Y() * L / 2.0 - xLoc.Y() * overhang,
        topZ - grooveDepth);
    // Cylinder axis along xLoc, radius kRootRadius
    const gp_Ax2 cylAx(cylStart, xLoc);
    const TopoDS_Shape rootCyl =
        pr::cylinder(cylAx, kRootRadius, cutLen);

    // ── Fuse all three cutters into one connected cutter ──────────────
    const TopoDS_Shape fusedAB    = pr::fuse(bevelACutter, bevelBCutter);
    const TopoDS_Shape fusedAll   = pr::fuse(fusedAB, rootCyl);
    const TopoDS_Shape newShape   = pr::cut(wp.shape(), fusedAll);

    // ── Build signature ────────────────────────────────────────────────
    json params = {
        { "entry_face_id",    *entryId },
        { "edge_dir",         { in.edge_dir.X(), in.edge_dir.Y(), in.edge_dir.Z() } },
        { "plate_t",          in.plate_t },
        { "groove_angle",     in.groove_angle },
        { "root_face_mm",     in.root_face_mm },
        { "edge_center_x_mm", in.edge_center_x_mm },
        { "edge_center_y_mm", in.edge_center_y_mm },
        { "edge_length_mm",   in.edge_length_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  3 },
        { "plate_t",           in.plate_t },
        { "groove_angle",      in.groove_angle },
        { "root_face_mm",      in.root_face_mm },
        { "groove_depth_mm",   grooveDepth },        // derived
        { "groove_top_width_mm", 2.0 * halfWidthTop }, // derived
        { "root_radius_mm",    kRootRadius },         // derived
        { "edge_length_mm",    in.edge_length_mm },
        { "edge_dir",          { in.edge_dir.X(), in.edge_dir.Y(), in.edge_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "bevel_mill;ball_mill";
    tooling.tool_dia_mm       = std::max(4.0, 2.0 * kRootRadius);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    // V-volume = grooveDepth × halfWidthTop × length (single bevel triangle)
    //            ×2 (two bevels) = grooveDepth × top_width × length.
    tooling.stock_removed_mm3 =
        grooveDepth * 2.0 * halfWidthTop * in.edge_length_mm;
    tooling.est_cycle_time_s  = std::max(2.0, in.edge_length_mm / 30.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "bevel_mill" }, { "pass",  "bevel_A" } },
            { { "tool_type", "bevel_mill" }, { "pass",  "bevel_B" } },
            { { "tool_type", "ball_mill"  }, { "pass",  "root_R"  },
              { "tool_dia_mm", 2.0 * kRootRadius } },
        } },
        { "aws_classification", "BUC1" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());   // carry prior history
    wpNew->addFeature(sig);

    spdlog::debug("skill::full_penetration_butt_prep applied: plate_t={} angle={} land={} L={}",
                  in.plate_t, in.groove_angle, in.root_face_mm, in.edge_length_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────
//
// Geometric signature of a single-V butt-prep groove:
//   - TWO planar faces with normals tilted from vertical by half(angle), and
//     mirrored along a shared horizontal "edge_dir" axis (their normals'
//     XY components are anti-parallel).
//   - the faces meet at a low-Z line — the groove root.
//   - groove vertical extent matches plate_t − root_face.
//
// Pure chamfer geometry is ambiguous with this; we discriminate by requiring
// the two angled faces to be SIGNIFICANTLY large (full plate depth, not a
// short break) and to share a common bottom Z within tolerance.

namespace {

struct AngledFace {
    int    faceIdx;
    gp_Dir normal;
    gp_Pnt center;
    double area;
    double minZ;
    double maxZ;
};

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness < 3.0) return out;

    // Collect candidate angled planar faces.
    std::vector<AngledFace> cand;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        const double nz = std::abs(n.Z());
        // Reject true verticals (chamfer side) and near-horizontals.
        if (nz < 0.1 || nz > 0.95) continue;
        // Need clear horizontal component (XY normal projection ≠ 0).
        const double nxy = std::hypot(n.X(), n.Y());
        if (nxy < 0.1) continue;

        AngledFace af;
        af.faceIdx = i;
        af.normal  = n;
        af.center  = wp.faceCenter(i);
        af.area    = wp.faceArea(i);
        // Z extent via bounding circle of face explorer
        double zLo = std::numeric_limits<double>::max();
        double zHi = std::numeric_limits<double>::lowest();
        for (TopExp_Explorer ev(wp.face(i), TopAbs_VERTEX); ev.More(); ev.Next()) {
            const TopoDS_Vertex& v = TopoDS::Vertex(ev.Current());
            const gp_Pnt p = BRep_Tool::Pnt(v);
            zLo = std::min(zLo, p.Z());
            zHi = std::max(zHi, p.Z());
        }
        af.minZ = zLo;
        af.maxZ = zHi;
        cand.push_back(af);
    }
    if (cand.size() < 2) return out;

    // Find a pair whose XY-projected normals are roughly anti-parallel and
    // whose bottom Z (the V root) is within tolerance.
    for (size_t i = 0; i < cand.size(); ++i) {
        for (size_t j = i + 1; j < cand.size(); ++j) {
            const auto& A = cand[i];
            const auto& B = cand[j];
            // XY normal anti-parallel?
            const gp_Vec naXY(A.normal.X(), A.normal.Y(), 0.0);
            const gp_Vec nbXY(B.normal.X(), B.normal.Y(), 0.0);
            const double maMag = naXY.Magnitude();
            const double mbMag = nbXY.Magnitude();
            if (maMag < 0.1 || mbMag < 0.1) continue;
            const double cosXY = naXY.Dot(nbXY) / (maMag * mbMag);
            if (cosXY > -0.9) continue;   // not opposite
            // Normals tilt by the same angle from vertical (Z-component magnitudes equal).
            if (std::abs(std::abs(A.normal.Z()) - std::abs(B.normal.Z())) > 0.1) continue;
            // Share the same root Z (bottom of V).
            if (std::abs(A.minZ - B.minZ) > 0.5) continue;
            // Root must be below the top by ≈ (plate_t − root_face).
            const double grooveDepth = zMax - A.minZ;
            if (grooveDepth < 1.0 || grooveDepth > thickness) continue;

            // Derive groove angle: each face tilts (90° − asin|nz|) from vertical.
            const double tiltDeg = std::acos(std::clamp(std::abs(A.normal.Z()), 0.0, 1.0))
                                   * 180.0 / M_PI;
            const double halfAngleFromVertical = 90.0 - tiltDeg;
            const double grooveAngle = 2.0 * halfAngleFromVertical;
            if (grooveAngle < 30.0 || grooveAngle > 90.0) continue;

            // Edge direction = perpendicular to XY normals.
            const gp_Dir edgeDir(-naXY.Y(), naXY.X(), 0.0);
            const gp_Pnt midPt(
                (A.center.X() + B.center.X()) * 0.5,
                (A.center.Y() + B.center.Y()) * 0.5,
                (A.center.Z() + B.center.Z()) * 0.5);
            // Edge length: greater of face Z-projected widths.
            const double edgeLen = std::max(
                std::hypot(A.center.X() - B.center.X(),
                           A.center.Y() - B.center.Y()) * 2.0,
                4.0);
            const double rootFace = std::max(0.5, thickness - grooveDepth);

            json recovered = {
                { "edge_dir",         { edgeDir.X(), edgeDir.Y(), edgeDir.Z() } },
                { "plate_t",          thickness },
                { "groove_angle",     grooveAngle },
                { "root_face_mm",     rootFace },
                { "edge_center_x_mm", midPt.X() },
                { "edge_center_y_mm", midPt.Y() },
                { "edge_length_mm",   edgeLen },
            };
            json matched = {
                { "source",      "geometric_fallback" },
                { "bevel_A_id",  A.faceIdx },
                { "bevel_B_id",  B.faceIdx },
                { "root_z",      A.minZ },
                { "groove_depth_mm", grooveDepth },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
        }
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& s : wp.features()) {
        if (s.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = s.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::full_penetration_butt_prep
