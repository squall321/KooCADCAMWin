// @lat: [[engine/skills#ball_valve_seat_compound]]

#include "ball_valve_seat_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Sphere.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::ball_valve_seat_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec table: API 608 standard ball-valve dimensions ──────────────────
//
// Reference: API 608 "Metal Ball Valves – Flanged, Threaded and Welding End",
// 6th ed., Table 4 ("Reduced Bore Ball Valves") + ISO 17292 §5.2.
//
// Fields (mm): ball_dia, port_dia, seat_id, seat_od, seat_depth,
//              stem_dia, stem_depth, flange_dia, flange_depth.
//
// These are nominal dimensions adapted for a 2-piece body; production
// values may differ per manufacturer but stay within ±10% of these.
struct BallValveSpec {
    const char* size;
    double      ball_dia;
    double      port_dia;
    double      seat_id;
    double      seat_od;
    double      seat_depth;
    double      stem_dia;
    double      stem_depth;
    double      flange_dia;
    double      flange_depth;
};

namespace {

constexpr std::array<BallValveSpec, 6> kBallValveTable {{
    { "DN15", 20.0, 12.0, 13.0, 19.0, 3.0,  8.0,  18.0, 28.0, 4.0 },
    { "DN20", 26.0, 16.0, 17.0, 25.0, 3.5, 10.0,  20.0, 36.0, 4.5 },
    { "DN25", 32.0, 20.0, 21.0, 31.0, 4.0, 12.0,  22.0, 44.0, 5.0 },
    { "DN32", 40.0, 25.0, 26.0, 39.0, 4.5, 14.0,  25.0, 54.0, 5.5 },
    { "DN40", 48.0, 32.0, 33.0, 47.0, 5.0, 16.0,  28.0, 64.0, 6.0 },
    { "DN50", 60.0, 40.0, 41.0, 59.0, 6.0, 20.0,  32.0, 78.0, 7.0 },
}};

const BallValveSpec* lookupSpec(const std::string& size) {
    for (const auto& s : kBallValveTable) {
        if (size == s.size) return &s;
    }
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const BallValveSpec* spec = lookupSpec(in.size_spec);
    if (!spec) {
        r.add("DFM-BALL-VALVE-SIZE", "error",
              "ball_valve_seat_compound: unknown size_spec '" + in.size_spec +
              "' (supported: DN15/DN20/DN25/DN32/DN40/DN50, API 608 Table 4)");
        return r;
    }

    const double ballDia   = in.ball_dia_mm    > 0.0 ? in.ball_dia_mm    : spec->ball_dia;
    const double portDia   = in.port_dia_mm    > 0.0 ? in.port_dia_mm    : spec->port_dia;
    const double seatId    = in.seat_groove_id_mm  > 0.0 ? in.seat_groove_id_mm  : spec->seat_id;
    const double seatOd    = in.seat_groove_od_mm  > 0.0 ? in.seat_groove_od_mm  : spec->seat_od;
    const double seatDepth = in.seat_groove_depth_mm > 0.0 ? in.seat_groove_depth_mm : spec->seat_depth;
    const double stemDia   = in.stem_bore_dia_mm   > 0.0 ? in.stem_bore_dia_mm   : spec->stem_dia;

    if (portDia >= ballDia) {
        r.add("DFM-BALL-VALVE-GEOM", "error",
              "port_dia must be < ball_dia (else through-bore breaks ball seat)");
    }
    if (seatId >= seatOd) {
        r.add("DFM-BALL-VALVE-SEAT", "error",
              "seat groove ID must be < OD");
    }
    if (seatOd > ballDia) {
        r.add("DFM-BALL-VALVE-SEAT", "warning",
              "seat OD exceeds ball dia — groove may break into cavity");
    }
    // ISO 17292 §6.3.4: PTFE seat depth ≥ 0.6 × seat_thickness (radial).
    const double seatThk = (seatOd - seatId) / 2.0;
    if (seatDepth < 0.6 * seatThk) {
        r.add("DFM-BALL-VALVE-SEAT-DEPTH", "error",
              "seat groove depth " + std::to_string(seatDepth) +
              " mm < 0.6 × seat thickness " + std::to_string(seatThk) +
              " mm (ISO 17292 §6.3.4 PTFE seat retention)");
    }
    if (in.ball_clearance_mm < 0.05 || in.ball_clearance_mm > 0.20) {
        r.add("DFM-BALL-VALVE-CLR", "error",
              "ball clearance " + std::to_string(in.ball_clearance_mm) +
              " mm outside [0.05, 0.20] mm (API 608 §6.7 torque-seal range)");
    }
    if (stemDia < 5.0) {
        r.add("DFM-BALL-VALVE-STEM", "error",
              "stem bore < 5 mm — insufficient torque capacity (API 608 §6.5)");
    }

    // Wall thickness sanity: stock must encompass cavity + flange.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cavityRadius = ballDia / 2.0 + in.ball_clearance_mm;
    if (xMax - xMin < 2.0 * cavityRadius + 10.0 ||
        yMax - yMin < 2.0 * cavityRadius + 10.0 ||
        zMax - zMin < 2.0 * cavityRadius + 10.0) {
        r.add("DFM-BALL-VALVE-STOCK", "error",
              "stock too small for cavity + 5 mm wall");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ball_valve_seat_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const BallValveSpec* spec = lookupSpec(in.size_spec);
    // (validate guarantees non-null)
    const double ballDia       = in.ball_dia_mm        > 0.0 ? in.ball_dia_mm        : spec->ball_dia;
    const double portDia       = in.port_dia_mm        > 0.0 ? in.port_dia_mm        : spec->port_dia;
    const double seatId        = in.seat_groove_id_mm  > 0.0 ? in.seat_groove_id_mm  : spec->seat_id;
    const double seatOd        = in.seat_groove_od_mm  > 0.0 ? in.seat_groove_od_mm  : spec->seat_od;
    const double seatDepth     = in.seat_groove_depth_mm > 0.0 ? in.seat_groove_depth_mm : spec->seat_depth;
    const double stemDia       = in.stem_bore_dia_mm     > 0.0 ? in.stem_bore_dia_mm     : spec->stem_dia;
    const double stemDepth     = in.stem_bore_depth_mm   > 0.0 ? in.stem_bore_depth_mm   : spec->stem_depth;
    const double flangeDia     = in.flange_recess_dia_mm   > 0.0 ? in.flange_recess_dia_mm   : spec->flange_dia;
    const double flangeDepth   = in.flange_recess_depth_mm > 0.0 ? in.flange_recess_depth_mm : spec->flange_depth;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.1;
    const gp_Pnt center(in.center_x_mm, in.center_y_mm, in.center_z_mm);

    // Slice-1: only +X flow, +Z stem accepted.
    if (std::abs(in.flow_axis.X() - 1.0) > 1e-6 ||
        std::abs(in.flow_axis.Y()) > 1e-6 ||
        std::abs(in.flow_axis.Z()) > 1e-6) {
        throw SkillError("ball_valve_seat_compound: slice-1 supports flow_axis=+X only");
    }
    if (std::abs(in.stem_axis.X()) > 1e-6 ||
        std::abs(in.stem_axis.Y()) > 1e-6 ||
        std::abs(in.stem_axis.Z() - 1.0) > 1e-6) {
        throw SkillError("ball_valve_seat_compound: slice-1 supports stem_axis=+Z only");
    }

    // ── Sub-feature 1: Spherical ball cavity ────────────────────────────
    const double cavityR = ballDia / 2.0 + in.ball_clearance_mm;
    BRepPrimAPI_MakeSphere sphMk(center, cavityR);
    sphMk.Build();
    if (!sphMk.IsDone()) throw SkillError("ball_valve_seat_compound: sphere build failed");
    const TopoDS_Shape ballCavity = sphMk.Shape();

    // ── Sub-feature 2: Through bore along flow (X) ──────────────────────
    const gp_Ax2 boreAx(
        gp_Pnt(xMin - kOverhang, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape throughBore = pr::cylinder(
        boreAx, portDia / 2.0, (xMax - xMin) + 2.0 * kOverhang);

    // ── Sub-features 3 & 4: PTFE seat-ring grooves (annular pockets) ────
    //
    // The left seat groove sits axially OUTBOARD of the ball cavity along
    // -X.  Its axial extent is `seatDepth`.  It is the annular volume
    // between seatId/2 and seatOd/2 — concentric with the bore.  We
    // construct each as outer-cyl ∪ inner-cyl-removed → effectively an
    // annular cylinder of axial length seatDepth.  Both seat tools use
    // pr::annularRing for clean topology.
    //
    // Position: the inner edge of the groove tangents the ball cavity (so
    // groove_outer_x = center.X() − cavityR − seatDepth, groove_inner_x =
    // center.X() − cavityR for the left side).
    const double leftGrooveStartX  = center.X() - cavityR - seatDepth;
    const double rightGrooveStartX = center.X() + cavityR;

    const gp_Ax2 leftSeatAx(
        gp_Pnt(leftGrooveStartX, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape leftSeatGroove = pr::annularRing(
        leftSeatAx, seatOd / 2.0, seatId / 2.0, seatDepth);

    const gp_Ax2 rightSeatAx(
        gp_Pnt(rightGrooveStartX, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape rightSeatGroove = pr::annularRing(
        rightSeatAx, seatOd / 2.0, seatId / 2.0, seatDepth);

    // ── Sub-feature 5: Stem bore (top, -Z direction) ────────────────────
    const gp_Ax2 stemAx(
        gp_Pnt(center.X(), center.Y(), zMax + kOverhang),
        gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape stemBore = pr::cylinder(
        stemAx, stemDia / 2.0, stemDepth + kOverhang);

    // ── Sub-feature 6: Body-bonnet flange recess ────────────────────────
    //
    // Concentric counterbore-style recess around the stem on the top face.
    const gp_Ax2 flangeAx(
        gp_Pnt(center.X(), center.Y(), zMax + kOverhang),
        gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape flangeRecess = pr::cylinder(
        flangeAx, flangeDia / 2.0, flangeDepth + kOverhang);

    // Fuse-then-cut group: bore + ball cavity (concentric overlap),
    // seat grooves (overlap with bore), stem + flange (concentric).
    const TopoDS_Shape cavityAndBore  = pr::fuse(ballCavity, throughBore);
    const TopoDS_Shape withLeftSeat   = pr::fuse(cavityAndBore, leftSeatGroove);
    const TopoDS_Shape withSeats      = pr::fuse(withLeftSeat, rightSeatGroove);
    const TopoDS_Shape stemFlange     = pr::fuse(stemBore, flangeRecess);
    const TopoDS_Shape allCutters     = pr::fuse(withSeats, stemFlange);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), allCutters);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",          in.center_x_mm },
        { "center_y_mm",          in.center_y_mm },
        { "center_z_mm",          in.center_z_mm },
        { "flow_axis",            { in.flow_axis.X(), in.flow_axis.Y(), in.flow_axis.Z() } },
        { "stem_axis",            { in.stem_axis.X(), in.stem_axis.Y(), in.stem_axis.Z() } },
        { "size_spec",            in.size_spec },
        { "ball_dia_mm",          ballDia },
        { "port_dia_mm",          portDia },
        { "seat_groove_id_mm",    seatId },
        { "seat_groove_od_mm",    seatOd },
        { "seat_groove_depth_mm", seatDepth },
        { "stem_bore_dia_mm",     stemDia },
        { "stem_bore_depth_mm",   stemDepth },
        { "flange_recess_dia_mm",   flangeDia },
        { "flange_recess_depth_mm", flangeDepth },
        { "ball_clearance_mm",    in.ball_clearance_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     6 },
        { "size_spec",            in.size_spec },
        { "subfeatures",          { "ball_cavity", "through_bore",
                                    "left_seat_groove", "right_seat_groove",
                                    "stem_bore", "flange_recess" } },
        { "spherical_face_count", 1 },
        { "cyl_face_count",       4 },   // bore + stem + flange + seat-IDs
        { "annular_seat_count",   2 },
        { "ball_dia_mm",          ballDia },
        { "port_dia_mm",          portDia },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "ball_end_mill;boring_bar;groove_tool";
    tooling.tool_dia_mm       = ballDia;
    tooling.tool_length_mm    = ballDia + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 320.0;
    tooling.feed_per_tooth_mm = 0.04;
    // Volume: sphere + bore − sphere∩bore + 2 × annular groove + stem + flange.
    const double sphV    = 4.0 / 3.0 * M_PI * std::pow(cavityR, 3);
    const double boreV   = M_PI * std::pow(portDia / 2.0, 2) * (xMax - xMin);
    const double seatV   = M_PI * (std::pow(seatOd / 2.0, 2) - std::pow(seatId / 2.0, 2)) * seatDepth;
    const double stemV   = M_PI * std::pow(stemDia / 2.0, 2) * stemDepth;
    const double flangeV = M_PI * std::pow(flangeDia / 2.0, 2) * flangeDepth;
    tooling.stock_removed_mm3 = sphV + boreV + 2.0 * seatV + stemV + flangeV;
    tooling.est_cycle_time_s  = std::max(12.0, tooling.stock_removed_mm3 / 1100.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" },    { "tool_dia_mm", portDia },
              { "note", "through bore" } },
            { { "tool_type", "ball_end_mill" }, { "tool_dia_mm", ballDia },
              { "note", "spherical ball cavity" } },
            { { "tool_type", "groove_tool" },   { "tool_dia_mm", seatOd },
              { "note", "PTFE seat grooves L+R" } },
            { { "tool_type", "drill" },         { "tool_dia_mm", stemDia },
              { "note", "stem bore" } },
            { { "tool_type", "end_mill" },      { "tool_dia_mm", flangeDia },
              { "note", "body-bonnet flange recess" } },
        } },
        { "standard", "API 608 Table 4 + ISO 17292 §6.3.4" },
        { "seat_material", "PTFE" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::ball_valve_seat_compound applied: spec={} ball={} port={} faces {}→{}",
        in.size_spec, ballDia, portDia, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // 1) Metadata replay (preferred — exact recovery)
    for (const auto& f : wp.features()) {
        if (f.skill_id == kSkillId) {
            RecognizedFeature r;
            r.skill_id         = kSkillId;
            r.recovered_params = f.params;
            r.confidence       = 1.0;
            r.matched_geometry = { { "source", "metadata_replay" } };
            out.push_back(r);
        }
    }
    if (!out.empty()) return out;

    // 2) Geometric heuristic — spatially correlate:
    //    spherical cavity center ↔ flow-axis cylinder axis ↔ vertical stem.
    //
    // The ball-valve signature requires the spherical cavity CENTER to be
    // on the flow-bore axis line, AND the stem cylinder axis to pass
    // through that same point.  These coaxiality constraints distinguish
    // a real ball-valve cavity from an unrelated sphere + cylinder pair.
    struct SphInfo { int idx; gp_Pnt center; double radius; };
    std::vector<SphInfo> spheres;
    std::vector<int> flowCyls;
    std::vector<double> flowR;
    std::vector<int> stemCyls;
    std::vector<double> stemR;
    std::vector<gp_Pnt> stemCenters;
    std::vector<gp_Ax1> flowAxes;

    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        const GeomAbs_SurfaceType st = s.GetType();
        if (st == GeomAbs_Sphere) {
            const gp_Sphere sph = s.Sphere();
            spheres.push_back({ i, sph.Location(), sph.Radius() });
        } else if (st == GeomAbs_Cylinder) {
            const gp_Cylinder cyl = s.Cylinder();
            const gp_Dir ad = cyl.Axis().Direction();
            if (std::abs(std::abs(ad.X()) - 1.0) < 5e-2) {
                flowCyls.push_back(i);
                flowR.push_back(cyl.Radius());
                flowAxes.push_back(cyl.Axis());
            } else if (std::abs(std::abs(ad.Z()) - 1.0) < 5e-2) {
                stemCyls.push_back(i);
                stemR.push_back(cyl.Radius());
                stemCenters.push_back(wp.faceCenter(i));
            }
        }
    }
    if (spheres.empty() || flowCyls.empty() || stemCyls.empty()) return out;

    // Pick the LARGEST sphere (the ball cavity, not chamfer fillets).
    auto biggestSph = std::max_element(
        spheres.begin(), spheres.end(),
        [](const SphInfo& a, const SphInfo& b){ return a.radius < b.radius; });
    const gp_Pnt sphCenter = biggestSph->center;
    const double sphRadius = biggestSph->radius;

    // Pick the flow cylinder whose axis comes closest to sphCenter
    // (coaxiality is what defines the ball-cavity / port-bore topology).
    auto distToLine = [](const gp_Pnt& p, const gp_Ax1& ax) {
        const gp_Vec v(ax.Location(), p);
        const gp_Vec d(ax.Direction());
        const gp_Vec proj = d * v.Dot(d);
        return (v - proj).Magnitude();
    };
    int bestFlow = -1;
    double bestFlowDist = 1e30;
    for (size_t k = 0; k < flowCyls.size(); ++k) {
        const double dd = distToLine(sphCenter, flowAxes[k]);
        if (dd < bestFlowDist) { bestFlowDist = dd; bestFlow = static_cast<int>(k); }
    }
    if (bestFlow < 0 || bestFlowDist > sphRadius + 5.0) return out;
    const double portDia = 2.0 * flowR[static_cast<size_t>(bestFlow)];

    // Pick the stem cylinder whose XY position is closest to (sphCenter.X,
    // sphCenter.Y).  That's the kinematic requirement of a ball valve: stem
    // drops onto the ball center.
    int bestStem = -1;
    double bestStemXY = 1e30;
    for (size_t k = 0; k < stemCyls.size(); ++k) {
        const double dxy = std::hypot(
            stemCenters[k].X() - sphCenter.X(),
            stemCenters[k].Y() - sphCenter.Y());
        if (dxy < bestStemXY) { bestStemXY = dxy; bestStem = static_cast<int>(k); }
    }
    if (bestStem < 0 || bestStemXY > sphRadius + 5.0) return out;
    const double stemDia = 2.0 * stemR[static_cast<size_t>(bestStem)];

    // Best-fit spec — pick the size whose ball_dia is closest to 2*sphRadius
    // (more reliable than port-based match because the cavity has clearance).
    const BallValveSpec* best = &kBallValveTable.front();
    double bestErr = std::abs(best->ball_dia - 2.0 * sphRadius);
    for (const auto& s : kBallValveTable) {
        const double err = std::abs(s.ball_dia - 2.0 * sphRadius);
        if (err < bestErr) { best = &s; bestErr = err; }
    }

    // Confidence rises with coaxial consistency.
    const double coaxQuality =
        std::max(0.0, 1.0 - (bestFlowDist + bestStemXY) / (sphRadius + 1e-3));
    const double confidence = std::min(0.90, 0.62 + 0.15 * coaxQuality);

    json recovered = {
        { "size_spec",        std::string(best->size) },
        { "ball_dia_mm",      2.0 * sphRadius },
        { "port_dia_mm",      portDia },
        { "stem_bore_dia_mm", stemDia },
        { "center_x_mm",      sphCenter.X() },
        { "center_y_mm",      sphCenter.Y() },
        { "center_z_mm",      sphCenter.Z() },
        { "flow_axis",        { 1.0, 0.0, 0.0 } },
        { "stem_axis",        { 0.0, 0.0, 1.0 } },
    };
    json matched = {
        { "spherical_face_count",  static_cast<int>(spheres.size()) },
        { "flow_cyl_face_id",      flowCyls[static_cast<size_t>(bestFlow)] },
        { "stem_cyl_face_id",      stemCyls[static_cast<size_t>(bestStem)] },
        { "ball_axis_offset_mm",   bestFlowDist },
        { "stem_axis_offset_mm",   bestStemXY },
        { "source",                "geometric" },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, confidence, matched });
    return out;
}

}  // namespace koocadcam::skill::ball_valve_seat_compound
