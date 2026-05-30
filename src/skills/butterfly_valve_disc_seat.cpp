// @lat: [[engine/skills#butterfly_valve_disc_seat]]

#include "butterfly_valve_disc_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

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

namespace koocadcam::skill::butterfly_valve_disc_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec table: API 609 standard butterfly-valve sizes ──────────────────
//
// API 609 "Butterfly Valves: Double Flanged, Lug- and Wafer-Type", Tables 3
// and 4 give body bore and stem dimensions for Class 150 service.
//
// Fields (mm): flow_bore, body_len, seat_groove_id, seat_groove_radial_depth,
//              seat_groove_width, stub_bearing_dia, stub_bearing_depth,
//              stem_port_dia.
struct BFVSpec {
    const char* size;
    double      flow_bore;
    double      body_len;
    double      seat_id;
    double      seat_r_depth;
    double      seat_width;
    double      stub_dia;
    double      stub_depth;
    double      stem_port_dia;
};

namespace {

constexpr std::array<BFVSpec, 6> kBFVTable {{
    { "DN50",  50.0,  45.0, 50.0, 2.5, 4.0, 12.0,  8.0, 10.0 },
    { "DN65",  65.0,  46.0, 65.0, 2.8, 4.5, 14.0,  9.0, 12.0 },
    { "DN80",  80.0,  46.0, 80.0, 3.0, 5.0, 16.0, 10.0, 14.0 },
    { "DN100",100.0,  52.0,100.0, 3.5, 5.5, 18.0, 12.0, 16.0 },
    { "DN125",125.0,  56.0,125.0, 4.0, 6.0, 20.0, 14.0, 18.0 },
    { "DN150",150.0,  56.0,150.0, 4.5, 6.5, 24.0, 16.0, 20.0 },
}};

const BFVSpec* lookupSpec(const std::string& size) {
    for (const auto& s : kBFVTable) {
        if (size == s.size) return &s;
    }
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.flow_bore_dia_mm <= 0.0 || in.flow_bore_len_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "butterfly_valve_disc_seat: flow bore dia/length must be > 0");
    }
    if (in.seat_groove_radial_depth_mm < 1.0) {
        r.add("DFM-BFV-SEAT-DEPTH", "error",
              "butterfly_valve_disc_seat: O-ring seat radial depth " +
              std::to_string(in.seat_groove_radial_depth_mm) +
              " mm < 1.0 mm minimum (resilient-seat retention)");
    }
    // API 609 §6 — disc-stem ratio: stub dia ≥ flow_bore × 0.15.
    if (in.stub_bearing_dia_mm < 0.15 * in.flow_bore_dia_mm) {
        r.add("DFM-BFV-STEM-RATIO", "error",
              "butterfly_valve_disc_seat: stub bearing dia " +
              std::to_string(in.stub_bearing_dia_mm) +
              " mm < 0.15 × bore (API 609 §6 disc-stem ratio)");
    }
    if (in.stem_port_dia_mm > in.stub_bearing_dia_mm) {
        r.add("DFM-BFV-PORT", "error",
              "butterfly_valve_disc_seat: stem port dia exceeds stub bearing "
              "(driver coupling won't fit in stub bearing)");
    }
    if (in.seat_groove_width_mm < 2.0 || in.seat_groove_width_mm > 12.0) {
        r.add("DFM-BFV-SEAT-WIDTH", "error",
              "butterfly_valve_disc_seat: seat groove width " +
              std::to_string(in.seat_groove_width_mm) +
              " mm outside [2, 12] mm (API 609 §6 seat ring spec)");
    }
    // O-ring groove must NOT intersect bearing stub axis — give ≥ 5 mm clear.
    if (in.seat_groove_width_mm > in.flow_bore_len_mm - 10.0) {
        r.add("DFM-BFV-CLR", "error",
              "butterfly_valve_disc_seat: seat groove leaves < 5 mm "
              "clearance to bearing stubs");
    }
    if (in.stub_bearing_depth_mm < 5.0) {
        r.add("DFM-BFV-STUB-DEPTH", "error",
              "butterfly_valve_disc_seat: stub bearing depth < 5 mm "
              "insufficient for stem journal length");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "butterfly_valve_disc_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    if (std::abs(in.flow_axis.X() - 1.0) > 1e-6 ||
        std::abs(in.flow_axis.Y()) > 1e-6 ||
        std::abs(in.flow_axis.Z()) > 1e-6) {
        throw SkillError("butterfly_valve_disc_seat: slice-1 supports flow_axis=+X only");
    }
    if (std::abs(in.stem_axis.X()) > 1e-6 ||
        std::abs(in.stem_axis.Y()) > 1e-6 ||
        std::abs(in.stem_axis.Z() - 1.0) > 1e-6) {
        throw SkillError("butterfly_valve_disc_seat: slice-1 supports stem_axis=+Z only");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.1;

    const gp_Pnt center(in.center_x_mm, in.center_y_mm, in.center_z_mm);

    // ── Sub-feature 1: Through cylindrical bore (axial along flow) ──────
    //
    // Build a flow-axial cylinder spanning the full body length.  We start
    // the cylinder at xMin-overhang to guarantee a clean break-through.
    const gp_Ax2 boreAx(
        gp_Pnt(xMin - kOverhang, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape throughBore = pr::cylinder(
        boreAx, in.flow_bore_dia_mm / 2.0,
        (xMax - xMin) + 2.0 * kOverhang);

    // ── Sub-feature 2: O-ring seat groove (annular, mid-length of bore) ─
    //
    // The seat is an annular groove on the bore wall: from the wall outward
    // (in radial sense) by seat_groove_radial_depth.  Implemented as an
    // annular ring of outer radius (bore/2 + depth) and inner radius bore/2,
    // axial extent seat_groove_width, centered axially on `center.X()`.
    const double seatOuterR = in.flow_bore_dia_mm / 2.0 + in.seat_groove_radial_depth_mm;
    const double seatInnerR = in.flow_bore_dia_mm / 2.0;
    const gp_Ax2 seatAx(
        gp_Pnt(center.X() - in.seat_groove_width_mm / 2.0, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape seatGroove = pr::annularRing(
        seatAx, seatOuterR, seatInnerR, in.seat_groove_width_mm);

    // ── Sub-features 3 & 4: Top + bottom stub bearings (vertical) ───────
    //
    // Top bearing — small cylinder going DOWNWARD from above the body, into
    // the body wall, stopping at depth stub_bearing_depth.  It penetrates
    // INTO the bore wall to host the stem journal.
    const gp_Ax2 topStubAx(
        gp_Pnt(center.X(), center.Y(), zMax + kOverhang),
        gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape topStub = pr::cylinder(
        topStubAx, in.stub_bearing_dia_mm / 2.0,
        in.stub_bearing_depth_mm + kOverhang);

    const gp_Ax2 botStubAx(
        gp_Pnt(center.X(), center.Y(), zMin - kOverhang),
        gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape botStub = pr::cylinder(
        botStubAx, in.stub_bearing_dia_mm / 2.0,
        in.stub_bearing_depth_mm + kOverhang);

    // ── Sub-feature 5: Top stem port (extends through to outer surface) ─
    //
    // A smaller cylinder coaxial with the stub bearing, going all the way
    // through the top wall.  It is the driver-coupling access.
    const gp_Ax2 topPortAx(
        gp_Pnt(center.X(), center.Y(), zMax + kOverhang),
        gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape topPort = pr::cylinder(
        topPortAx, in.stem_port_dia_mm / 2.0,
        (zMax - zMin) + 2.0 * kOverhang);  // full through

    // Bottom port (mirror) — keeps geometry symmetric, allows drainage.
    const gp_Ax2 botPortAx(
        gp_Pnt(center.X(), center.Y(), zMin - kOverhang),
        gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape botPort = pr::cylinder(
        botPortAx, in.stem_port_dia_mm / 2.0,
        (zMax - zMin) + 2.0 * kOverhang);

    // ── Fuse and cut ────────────────────────────────────────────────────
    const TopoDS_Shape boreAndSeat   = pr::fuse(throughBore, seatGroove);
    const TopoDS_Shape topGroup      = pr::fuse(topStub, topPort);
    const TopoDS_Shape botGroup      = pr::fuse(botStub, botPort);
    const TopoDS_Shape verticals     = pr::fuse(topGroup, botGroup);
    const TopoDS_Shape allCutters    = pr::fuse(boreAndSeat, verticals);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), allCutters);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",          in.center_x_mm },
        { "center_y_mm",          in.center_y_mm },
        { "center_z_mm",          in.center_z_mm },
        { "flow_axis",            { in.flow_axis.X(), in.flow_axis.Y(), in.flow_axis.Z() } },
        { "stem_axis",            { in.stem_axis.X(), in.stem_axis.Y(), in.stem_axis.Z() } },
        { "flow_bore_dia_mm",     in.flow_bore_dia_mm },
        { "flow_bore_len_mm",     in.flow_bore_len_mm },
        { "seat_groove_id_mm",    in.seat_groove_id_mm },
        { "seat_groove_radial_depth_mm", in.seat_groove_radial_depth_mm },
        { "seat_groove_width_mm", in.seat_groove_width_mm },
        { "stub_bearing_dia_mm",  in.stub_bearing_dia_mm },
        { "stub_bearing_depth_mm",in.stub_bearing_depth_mm },
        { "stem_port_dia_mm",     in.stem_port_dia_mm },
        { "body_y_span_mm",       in.body_y_span_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     5 },
        { "subfeatures",          { "through_bore", "seat_groove",
                                    "top_stub_bearing", "bottom_stub_bearing",
                                    "stem_port" } },
        { "flow_cyl_face_count",  1 },
        { "annular_seat_count",   1 },
        { "vertical_cyl_count",   4 },  // top+bot stubs + top+bot ports
        { "flow_bore_dia_mm",     in.flow_bore_dia_mm },
        { "stub_bearing_dia_mm",  in.stub_bearing_dia_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;groove_tool;drill";
    tooling.tool_dia_mm       = in.flow_bore_dia_mm;
    tooling.tool_length_mm    = in.flow_bore_len_mm + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double boreV = M_PI * std::pow(in.flow_bore_dia_mm / 2.0, 2)
                              * in.flow_bore_len_mm;
    const double seatV = M_PI * (std::pow(seatOuterR, 2) - std::pow(seatInnerR, 2))
                              * in.seat_groove_width_mm;
    const double stubV = 2.0 * M_PI * std::pow(in.stub_bearing_dia_mm / 2.0, 2)
                              * in.stub_bearing_depth_mm;
    const double portV = 2.0 * M_PI * std::pow(in.stem_port_dia_mm / 2.0, 2)
                              * ((zMax - zMin) / 2.0);
    tooling.stock_removed_mm3 = boreV + seatV + stubV + portV;
    tooling.est_cycle_time_s  = std::max(8.0, tooling.stock_removed_mm3 / 1200.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" }, { "tool_dia_mm", in.flow_bore_dia_mm },
              { "note", "through bore" } },
            { { "tool_type", "groove_tool" }, { "tool_dia_mm", in.seat_groove_radial_depth_mm * 2.0 },
              { "note", "O-ring seat groove" } },
            { { "tool_type", "drill" },   { "tool_dia_mm", in.stub_bearing_dia_mm },
              { "note", "top + bottom stub bearings" } },
            { { "tool_type", "drill" },   { "tool_dia_mm", in.stem_port_dia_mm },
              { "note", "top + bottom stem ports" } },
        } },
        { "standard", "API 609 §6" },
        { "seat_material", "EPDM" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::butterfly_valve_disc_seat applied: bore={} stub={} faces {}→{}",
        in.flow_bore_dia_mm, in.stub_bearing_dia_mm,
        wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay
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

    // Geometric: largest flow-axis cylinder + a vertical TOP and BOTTOM
    // stub-bearing pair whose XY position sits on the bore axis (kinematic
    // constraint of a butterfly valve: stem pierces the bore through the disc).
    int flowCyl = -1;
    double flowR = 0.0;
    gp_Ax1 flowAxis;
    struct VC { int idx; double radius; gp_Pnt center; };
    std::vector<VC> verticalCyls;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder cyl = s.Cylinder();
        const gp_Dir ad = cyl.Axis().Direction();
        const double r = cyl.Radius();
        if (std::abs(std::abs(ad.X()) - 1.0) < 5e-2) {
            // flow-axis: pick the LARGEST cylinder as the flow bore.
            if (r > flowR) { flowCyl = i; flowR = r; flowAxis = cyl.Axis(); }
        } else if (std::abs(std::abs(ad.Z()) - 1.0) < 5e-2) {
            verticalCyls.push_back({ i, r, wp.faceCenter(i) });
        }
    }
    if (flowCyl < 0 || verticalCyls.size() < 2) return out;

    // Filter vertical cylinders to those whose XY position lies on the flow
    // axis (within bore radius + slack).  These are the stub bearings / stem
    // ports — they MUST coincide with the bore axis or it isn't a BFV.
    auto onBoreAxis = [&](const gp_Pnt& c) {
        const double dxy = std::hypot(
            c.X() - flowAxis.Location().X(),
            c.Y() - flowAxis.Location().Y());
        return dxy <= flowR + 1.0;
    };
    std::vector<VC> onAxis;
    for (const auto& v : verticalCyls) if (onBoreAxis(v.center)) onAxis.push_back(v);
    if (onAxis.size() < 2) return out;

    // Separate into TOP (z > bore center) and BOTTOM groups.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zMid = flowAxis.Location().Z();
    bool hasTop = false, hasBot = false;
    for (const auto& v : onAxis) {
        if (v.center.Z() > zMid) hasTop = true;
        else                     hasBot = true;
    }

    // Stub bearing dia = the LARGER of distinct radii among on-axis verticals
    // (stem ports are smaller; bearings are bigger).
    std::vector<double> radii;
    for (const auto& v : onAxis) radii.push_back(v.radius);
    std::sort(radii.begin(), radii.end());
    const double medStub = radii[radii.size() / 2];
    const double maxStub = radii.back();

    int subScore = 0;
    if (true)    ++subScore;        // through bore
    if (hasTop)  ++subScore;        // top stub
    if (hasBot)  ++subScore;        // bottom stub
    if (onAxis.size() >= 4) ++subScore;   // top+bot ports too
    const double confidence = std::min(0.90, 0.55 + 0.08 * subScore);

    json recovered = {
        { "flow_bore_dia_mm",    2.0 * flowR },
        { "stub_bearing_dia_mm", 2.0 * maxStub },
        { "stem_port_dia_mm",    2.0 * medStub },
        { "center_x_mm",         flowAxis.Location().X() },
        { "center_y_mm",         flowAxis.Location().Y() },
        { "center_z_mm",         zMid },
        { "flow_axis",           { 1.0, 0.0, 0.0 } },
        { "stem_axis",           { 0.0, 0.0, 1.0 } },
    };
    json matched = {
        { "flow_cyl_face_id",      flowCyl },
        { "on_axis_vertical_count", static_cast<int>(onAxis.size()) },
        { "has_top_stub",           hasTop },
        { "has_bottom_stub",        hasBot },
        { "subfeature_score",       subScore },
        { "source",                 "geometric" },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, confidence, matched });
    return out;
}

}  // namespace koocadcam::skill::butterfly_valve_disc_seat
