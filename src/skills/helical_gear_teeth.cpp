// @lat: [[engine/skills#helical_gear_teeth]]

#include "helical_gear_teeth.hpp"

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

namespace koocadcam::skill::helical_gear_teeth {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

int deriveToothCount(double pitch_dia_mm, double normal_module_mm)
{
    if (normal_module_mm <= 0.0) return 0;
    return static_cast<int>(std::round(pitch_dia_mm / normal_module_mm));
}

// Build an oblique-prism gullet cutter: a planar gullet wedge at z = bottom
// extruded by a vector that combines axial Z motion + tangential offset
// (equal to Z * tan(helix)).  The result is the swept gullet of a helical
// tooth — approximate but volumetrically accurate.
TopoDS_Shape buildHelicalGulletCutter(int i, int N, double moduleM, double pitchR,
                                      double thickness, double overhang,
                                      double helixDeg, bool rightHand)
{
    const double toothAngle = 2.0 * M_PI / static_cast<double>(N);
    const double th0        = (static_cast<double>(i) + 0.5) * toothAngle;

    const double toothArc   = M_PI * moduleM / 2.0;
    const double toothAng   = toothArc / pitchR;
    const double gulletAng  = std::max(toothAngle - toothAng, toothAngle * 0.3);

    const double rRoot = std::max(0.1, pitchR - 1.25 * moduleM);
    const double rOut  = pitchR + 1.00 * moduleM;

    const double a0 = th0 - gulletAng / 2.0;
    const double a1 = th0 + gulletAng / 2.0;

    const int kFlankPts = 5;
    std::vector<gp_Pnt> leftFlank(kFlankPts);
    std::vector<gp_Pnt> rightFlank(kFlankPts);
    for (int k = 0; k < kFlankPts; ++k) {
        const double t  = static_cast<double>(k) / static_cast<double>(kFlankPts - 1);
        const double r  = rRoot + t * (rOut - rRoot);
        const double bias = 0.25 * gulletAng * std::sin(t * M_PI);
        const double aL   = a0 + bias;
        const double aR   = a1 - bias;
        leftFlank[k]  = gp_Pnt(r * std::cos(aL), r * std::sin(aL), -overhang);
        rightFlank[k] = gp_Pnt(r * std::cos(aR), r * std::sin(aR), -overhang);
    }

    BRepBuilderAPI_MakeWire wire;
    for (int k = 0; k < kFlankPts - 1; ++k)
        wire.Add(BRepBuilderAPI_MakeEdge(leftFlank[k], leftFlank[k + 1]).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(leftFlank[kFlankPts - 1], rightFlank[kFlankPts - 1]).Edge());
    for (int k = kFlankPts - 1; k > 0; --k)
        wire.Add(BRepBuilderAPI_MakeEdge(rightFlank[k], rightFlank[k - 1]).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(rightFlank[0], leftFlank[0]).Edge());

    if (!wire.IsDone())
        throw Standard_Failure("helical_gear_teeth: wire build failed");

    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone())
        throw Standard_Failure("helical_gear_teeth: face build failed");

    // Extrude along oblique vector — direction tilted by helix_angle around
    // the gullet center.  The tangential offset for the upper section equals
    // Z * tan(helix); decompose that into local XY based on th0.
    const double H = thickness + 2.0 * overhang;
    const double tanH = std::tan(helixDeg * M_PI / 180.0);
    const double tangOffset = H * tanH;
    const double sign = rightHand ? 1.0 : -1.0;
    // Tangential direction at th0 (perpendicular to radial): (-sin th0, cos th0).
    const double dx = -std::sin(th0) * tangOffset * sign;
    const double dy =  std::cos(th0) * tangOffset * sign;

    BRepPrimAPI_MakePrism prism(face.Face(), gp_Vec(dx, dy, H));
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("helical_gear_teeth: prism build failed");

    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.blank_dia_mm <= 0.0 || in.blank_thick_mm <= 0.0 ||
        in.normal_module_mm <= 0.0 || in.pitch_dia_mm <= 0.0 ||
        in.bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "helical_gear_teeth: positive blank/module/pitch/bore required");
        return r;
    }

    if (in.normal_module_mm < 0.5 || in.normal_module_mm > 10.0) {
        r.add("DFM-GEAR-MODULE", "error",
              "helical_gear_teeth: normal_module outside [0.5, 10]");
    }

    const int N = deriveToothCount(in.pitch_dia_mm, in.normal_module_mm);
    if (N < 6 || N > 200) {
        r.add("DFM-GEAR-COUNT", "error",
              "helical_gear_teeth: derived tooth count " +
              std::to_string(N) + " outside [6, 200]");
    }

    if (in.helix_angle_deg < 5.0 || in.helix_angle_deg > 45.0) {
        r.add("DFM-GEAR-HELIX", "error",
              "helical_gear_teeth: helix_angle " +
              std::to_string(in.helix_angle_deg) + " outside [5, 45]");
    }

    const double od = (static_cast<double>(N) + 2.0) * in.normal_module_mm;
    if (od > in.blank_dia_mm + 1e-3) {
        r.add("DFM-GEAR-OD", "error",
              "helical_gear_teeth: derived OD " + std::to_string(od) +
              " mm exceeds blank_dia " + std::to_string(in.blank_dia_mm));
    }

    // AGMA face-width minimum: 2 normal pitches engaged.
    const double normalPitch = M_PI * in.normal_module_mm;
    if (in.blank_thick_mm < 2.0 * normalPitch) {
        r.add("DFM-GEAR-FW", "warning",
              "helical_gear_teeth: face_width " +
              std::to_string(in.blank_thick_mm) +
              " mm < 2 normal pitches (" + std::to_string(2.0 * normalPitch) +
              ") — limited helical overlap");
    }

    if (in.bore_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "helical_gear_teeth: bore_dia < 0.8 mm");
    }
    if (in.bore_dia_mm >= in.pitch_dia_mm - 2.0 * in.normal_module_mm) {
        r.add("DFM-GEAR-BORE", "error",
              "helical_gear_teeth: bore_dia collides with tooth root circle");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "helical_gear_teeth DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int    N      = deriveToothCount(in.pitch_dia_mm, in.normal_module_mm);
    const double pitchR = in.pitch_dia_mm / 2.0;
    const double boreR  = in.bore_dia_mm  / 2.0;
    const double Z      = in.blank_thick_mm;
    const double m      = in.normal_module_mm;
    const double overhang = 0.05;

    // Tool 1: through bore.
    const gp_Ax2 axBore(gp_Pnt(0, 0, -overhang), gp::DZ());
    const TopoDS_Shape boreTool = pr::cylinder(axBore, boreR, Z + 2.0 * overhang);

    // Tools 2..N+1: helical gullet cutters.
    std::vector<TopoDS_Shape> tools;
    tools.reserve(static_cast<size_t>(N) + 1);
    tools.push_back(boreTool);
    for (int i = 0; i < N; ++i) {
        tools.push_back(buildHelicalGulletCutter(
            i, N, m, pitchR, Z, overhang,
            in.helix_angle_deg, in.right_hand));
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    const double od = (static_cast<double>(N) + 2.0) * m;
    const int subCount = 1 + N;

    json params = {
        { "blank_dia_mm",        in.blank_dia_mm },
        { "blank_thick_mm",      in.blank_thick_mm },
        { "normal_module_mm",    in.normal_module_mm },
        { "pitch_dia_mm",        in.pitch_dia_mm },
        { "helix_angle_deg",     in.helix_angle_deg },
        { "pressure_angle_deg",  in.pressure_angle_deg },
        { "bore_dia_mm",         in.bore_dia_mm },
        { "right_hand",          in.right_hand },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     subCount },
        { "blank_dia_mm",         in.blank_dia_mm },
        { "blank_thick_mm",       in.blank_thick_mm },
        { "normal_module_mm",     in.normal_module_mm },
        { "pitch_dia_mm",         in.pitch_dia_mm },
        { "outer_dia_mm",         od },
        { "num_teeth",            N },
        { "bore_dia_mm",          in.bore_dia_mm },
        { "helix_angle_deg",      in.helix_angle_deg },
        { "pressure_angle_deg",   in.pressure_angle_deg },
        { "right_hand",           in.right_hand },
        { "standard",             "AGMA 2005-D03" },
    };

    ToolingMeta tooling;
    tooling.tool_type    = "gear_hob;drill";
    tooling.tool_dia_mm  = in.bore_dia_mm;
    tooling.tool_length_mm = in.blank_thick_mm + 10.0;
    tooling.tool_material = "hss";
    tooling.cutting_speed_sfm = 80.0;       // helical hobs run slower
    const double rRoot = pitchR - 1.25 * m;
    const double rOut  = od / 2.0;
    const double gulletWedgeVol = (rOut * rOut - rRoot * rRoot) * 0.5 *
                                  (2.0 * M_PI / N) * 0.6 * Z;
    tooling.stock_removed_mm3 =
        M_PI * boreR * boreR * Z + N * gulletWedgeVol;
    tooling.est_cycle_time_s = std::max(45.0, N * 3.0 + 90.0);
    tooling.extra = {
        { "synthesis_method", "oblique_prism_approx" },
        { "tool_sequence", {
            { { "tool_type", "drill" }, { "tool_dia_mm", in.bore_dia_mm } },
            { { "tool_type", "gear_hob" },
              { "normal_module_mm", m },
              { "num_teeth",        N },
              { "helix_angle_deg",  in.helix_angle_deg },
              { "hand",             in.right_hand ? "right" : "left" } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::helical_gear_teeth applied: m={} N={} helix={}deg",
                  m, N, in.helix_angle_deg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Geometric primary: presence of a bore + many oblique side faces.
    std::vector<double> innerCylRadii;
    int obliqueFlankCount = 0;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;

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
                // Helical tooth flanks have normal with small Z (they are
                // tilted from pure vertical by helix angle).
                if (std::abs(n.Z()) < 0.45 && std::abs(n.Z()) > 0.0) {
                    ++obliqueFlankCount;
                }
            } catch (...) {}
        }
    }

    if (!innerCylRadii.empty() && obliqueFlankCount >= 12) {
        std::sort(innerCylRadii.begin(), innerCylRadii.end());
        const double boreR = innerCylRadii.front();
        const int Nest = obliqueFlankCount / 2;
        if (Nest >= 6 && Nest <= 200) {
            const double od = std::max({ xMax - xMin, yMax - yMin });
            const double pitchDia = od * static_cast<double>(Nest) /
                                    (static_cast<double>(Nest) + 2.0);
            const double moduleEst = pitchDia / static_cast<double>(Nest);

            json recovered = {
                { "blank_dia_mm",        od },
                { "blank_thick_mm",      thickness },
                { "normal_module_mm",    moduleEst },
                { "pitch_dia_mm",        pitchDia },
                { "helix_angle_deg",     15.0 },
                { "pressure_angle_deg",  20.0 },
                { "bore_dia_mm",         2.0 * boreR },
                { "right_hand",          true },
            };
            json matched = {
                { "source",                "geometric_heuristic" },
                { "oblique_flank_count",   obliqueFlankCount },
                { "estimated_num_teeth",   Nest },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, 0.6, matched
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

}  // namespace koocadcam::skill::helical_gear_teeth
