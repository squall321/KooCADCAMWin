// @lat: [[engine/skills#ratchet_pawl_set]]

#include "ratchet_pawl_set.hpp"

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
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::ratchet_pawl_set {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Build an asymmetric tooth-gap cutter.  The gap is a wedge sweeping from
// the tip circle (rOut) to root circle (rRoot), with a steep flank on one
// side and shallow flank on the other.  In top-down view the gap looks like
// a saw-tooth notch.
//
//   pTipFront(steep_side, at rOut, angle = th0 - half_pitch)
//   pRootSteep (rRoot at offset determined by steep_flank_deg)
//   pRootShallow (rRoot at offset determined by shallow_flank_deg)
//   pTipBack (at rOut, angle = th0 + half_pitch)
//
// We approximate with a 4-point planar wire (steep flank, root edge,
// shallow flank, top tip arc closure).
TopoDS_Shape buildToothGapCutter(int i, int N, double rOut, double rRoot,
                                 double steep_deg, double shallow_deg,
                                 double thickness, double overhang,
                                 bool ccwLocking)
{
    const double pitchAng = 2.0 * M_PI / static_cast<double>(N);
    const double th0      = (static_cast<double>(i) + 0.5) * pitchAng;

    // Half pitch angle for the gap span on the OD.
    const double halfArc  = pitchAng * 0.5;
    // Convert flank angles to horizontal offsets at root level.
    // depth = rOut - rRoot.  At the root, the steep flank is offset
    // tangentially by depth * cot(steep) from the steep tip; shallow
    // similarly.
    const double depth    = rOut - rRoot;
    const double steepRad = steep_deg   * M_PI / 180.0;
    const double shallRad = shallow_deg * M_PI / 180.0;

    const double steepOffsetArc =
        std::min(halfArc * 1.6, depth / std::tan(steepRad) / rOut);
    const double shallOffsetArc =
        std::min(halfArc * 1.6, depth / std::tan(shallRad) / rOut);

    // Decide which side is "steep" vs "shallow" based on locking direction.
    // CCW-locking means rotation in +theta direction is BLOCKED — so the
    // pawl meets the steep flank on the LEADING (CCW-most) side.
    const double sign = ccwLocking ? 1.0 : -1.0;

    // Tip corners
    const double aTipSteep   = th0 - halfArc * sign;
    const double aTipShallow = th0 + halfArc * sign;

    // Root corners offset inward toward th0 by the tangential offsets
    const double aRootSteep   = aTipSteep   + steepOffsetArc * sign;
    const double aRootShallow = aTipShallow - shallOffsetArc * sign;

    const gp_Pnt pTipS(rOut  * std::cos(aTipSteep),   rOut  * std::sin(aTipSteep),
                       -overhang);
    const gp_Pnt pRootS(rRoot * std::cos(aRootSteep), rRoot * std::sin(aRootSteep),
                        -overhang);
    const gp_Pnt pRootH(rRoot * std::cos(aRootShallow), rRoot * std::sin(aRootShallow),
                        -overhang);
    const gp_Pnt pTipH(rOut  * std::cos(aTipShallow), rOut  * std::sin(aTipShallow),
                       -overhang);

    BRepBuilderAPI_MakeWire wire;
    wire.Add(BRepBuilderAPI_MakeEdge(pTipS,  pRootS).Edge());   // steep flank
    wire.Add(BRepBuilderAPI_MakeEdge(pRootS, pRootH).Edge());   // root edge
    wire.Add(BRepBuilderAPI_MakeEdge(pRootH, pTipH).Edge());    // shallow flank
    wire.Add(BRepBuilderAPI_MakeEdge(pTipH,  pTipS).Edge());    // tip closure
    if (!wire.IsDone())
        throw Standard_Failure("ratchet_pawl_set: wire build failed");

    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone())
        throw Standard_Failure("ratchet_pawl_set: face build failed");

    BRepPrimAPI_MakePrism prism(face.Face(),
        gp_Vec(0.0, 0.0, thickness + 2.0 * overhang));
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("ratchet_pawl_set: prism build failed");

    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wheel_dia_mm <= 0.0 || in.wheel_thick_mm <= 0.0 ||
        in.bore_dia_mm <= 0.0 || in.tooth_depth_mm <= 0.0 ||
        in.num_teeth <= 0) {
        r.add("DFM-INPUT", "error",
              "ratchet_pawl_set: positive wheel/bore/teeth required");
        return r;
    }

    if (in.num_teeth < 6 || in.num_teeth > 60) {
        r.add("DFM-RATCHET-N", "error",
              "ratchet_pawl_set: num_teeth " +
              std::to_string(in.num_teeth) + " outside [6, 60]");
    }

    if (in.tooth_depth_mm > 0.3 * in.wheel_dia_mm) {
        r.add("DFM-RATCHET-DEPTH", "error",
              "ratchet_pawl_set: tooth_depth " +
              std::to_string(in.tooth_depth_mm) +
              " mm > 0.3 * wheel_dia (collides with bore)");
    }

    if (in.steep_flank_deg < 70.0 || in.steep_flank_deg > 90.0) {
        r.add("DFM-RATCHET-STEEP", "error",
              "ratchet_pawl_set: steep_flank_deg " +
              std::to_string(in.steep_flank_deg) + " outside [70, 90]");
    }
    if (in.shallow_flank_deg < 20.0 || in.shallow_flank_deg > 60.0) {
        r.add("DFM-RATCHET-SHALLOW", "error",
              "ratchet_pawl_set: shallow_flank_deg " +
              std::to_string(in.shallow_flank_deg) + " outside [20, 60]");
    }

    const double rootDia = in.wheel_dia_mm - 2.0 * in.tooth_depth_mm;
    if (in.bore_dia_mm >= rootDia - 1.0) {
        r.add("DFM-RATCHET-BORE", "error",
              "ratchet_pawl_set: bore_dia " +
              std::to_string(in.bore_dia_mm) +
              " collides with tooth root circle " + std::to_string(rootDia));
    }

    if (in.bore_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "ratchet_pawl_set: bore_dia < 0.8 mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ratchet_pawl_set DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int    N       = in.num_teeth;
    const double rOut    = in.wheel_dia_mm  / 2.0;
    const double rRoot   = rOut - in.tooth_depth_mm;
    const double boreR   = in.bore_dia_mm   / 2.0;
    const double Z       = in.wheel_thick_mm;
    const double overhang = 0.05;

    // Tool 1: through bore.
    const gp_Ax2 axBore(gp_Pnt(0, 0, -overhang), gp::DZ());
    const TopoDS_Shape boreTool = pr::cylinder(axBore, boreR, Z + 2.0 * overhang);

    // Tools 2..N+1: N tooth-gap cutters.
    std::vector<TopoDS_Shape> tools;
    tools.reserve(static_cast<size_t>(N) + 1);
    tools.push_back(boreTool);
    for (int i = 0; i < N; ++i) {
        tools.push_back(buildToothGapCutter(
            i, N, rOut + overhang, rRoot, in.steep_flank_deg, in.shallow_flank_deg,
            Z, overhang, in.ccw_locking));
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    const int subCount = 1 + N;

    // Pawl dimensions (returned as metadata):
    //   length ≈ tooth_depth * 3
    //   width  ≈ tooth_depth * 0.8
    //   pawl tip at (wheel_dia/2 + 1mm, 0, Z/2)
    const double pawlLen = in.tooth_depth_mm * 3.0;
    const double pawlWid = in.tooth_depth_mm * 0.8;

    json params = {
        { "wheel_dia_mm",       in.wheel_dia_mm },
        { "wheel_thick_mm",     in.wheel_thick_mm },
        { "num_teeth",          in.num_teeth },
        { "tooth_depth_mm",     in.tooth_depth_mm },
        { "steep_flank_deg",    in.steep_flank_deg },
        { "shallow_flank_deg",  in.shallow_flank_deg },
        { "bore_dia_mm",        in.bore_dia_mm },
        { "ccw_locking",        in.ccw_locking },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     subCount },
        { "wheel_dia_mm",         in.wheel_dia_mm },
        { "wheel_thick_mm",       in.wheel_thick_mm },
        { "num_teeth",            N },
        { "tooth_depth_mm",       in.tooth_depth_mm },
        { "root_dia_mm",          in.wheel_dia_mm - 2.0 * in.tooth_depth_mm },
        { "bore_dia_mm",          in.bore_dia_mm },
        { "steep_flank_deg",      in.steep_flank_deg },
        { "shallow_flank_deg",    in.shallow_flank_deg },
        { "ccw_locking",          in.ccw_locking },
        { "is_asymmetric",        true },
        { "standard",             "agent-chosen: ANSI ratchet practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type    = "form_cutter;drill";
    tooling.tool_dia_mm  = in.tooth_depth_mm * 0.8;
    tooling.tool_material = "carbide";
    tooling.cutting_speed_sfm = 200.0;
    // Tooth gap volume estimate: trapezoidal area * thickness.
    const double tipArc   = 2.0 * M_PI / static_cast<double>(N) * rOut;
    const double rootArc  = std::max(0.0, tipArc - 0.5 * in.tooth_depth_mm);
    const double gapArea  = 0.5 * (tipArc + rootArc) * in.tooth_depth_mm;
    tooling.stock_removed_mm3 =
        M_PI * boreR * boreR * Z + N * gapArea * Z;
    tooling.est_cycle_time_s = std::max(20.0, N * 1.5 + 30.0);
    tooling.extra = {
        { "pawl", {
            { "type",      "rectangular_block" },
            { "length_mm", pawlLen },
            { "width_mm",  pawlWid },
            { "height_mm", in.wheel_thick_mm },
            { "tip_position",
                { rOut + 1.0, 0.0, Z / 2.0 } },
            { "note",
              "pawl is a separate piece; not cut into wheel; "
              "engages steep flank when wheel rotates in locked direction" },
        } },
        { "tool_sequence", {
            { { "tool_type", "drill" },        { "tool_dia_mm", in.bore_dia_mm } },
            { { "tool_type", "form_cutter" },  { "pass_count", N },
              { "note", "asymmetric ratchet form-cutter" } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::ratchet_pawl_set applied: N={} depth={} steep={}deg",
                  N, in.tooth_depth_mm, in.steep_flank_deg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Primary geometric: bore cyl + many side faces (non-horizontal planar)
// arranged with rotational symmetry.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    std::vector<double> innerCylRadii;
    int sideFaceCount = 0;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thick = zMax - zMin;
    const double dia   = std::max(xMax - xMin, yMax - yMin);

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFaceCylinder(i)) {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder cy = s.Cylinder();
            const gp_Dir d = cy.Axis().Direction();
            if (std::abs(std::abs(d.Z()) - 1.0) < 1e-2) {
                innerCylRadii.push_back(cy.Radius());
            }
        } else if (wp.isFacePlanar(i)) {
            try {
                gp_Dir n = wp.faceNormal(i);
                if (std::abs(n.Z()) < 0.3) {
                    ++sideFaceCount;
                }
            } catch (...) {}
        }
    }

    if (!innerCylRadii.empty() && sideFaceCount >= 12) {
        std::sort(innerCylRadii.begin(), innerCylRadii.end());
        const double boreR = innerCylRadii.front();
        // Each tooth-gap leaves ~3 planar walls (steep, root, shallow).
        const int Nest = sideFaceCount / 3;
        if (Nest >= 6 && Nest <= 60) {
            const double toothDepthEst = dia * 0.1;     // rule of thumb 10%
            json recovered = {
                { "wheel_dia_mm",       dia },
                { "wheel_thick_mm",     thick },
                { "num_teeth",          Nest },
                { "tooth_depth_mm",     toothDepthEst },
                { "steep_flank_deg",    85.0 },
                { "shallow_flank_deg",  30.0 },
                { "bore_dia_mm",        2.0 * boreR },
                { "ccw_locking",        true },
            };
            json matched = {
                { "source",            "geometric_heuristic" },
                { "side_face_count",   sideFaceCount },
                { "estimated_n",       Nest },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, 0.55, matched
            });
        }
    }

    // Fallback metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", f.pattern.value("subfeature_count", 0) },
            { "num_teeth",        f.pattern.value("num_teeth", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::ratchet_pawl_set
