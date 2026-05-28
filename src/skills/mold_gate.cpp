// @lat: [[engine/skills#mold_gate]]

#include "mold_gate.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::mold_gate {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

bool isPinGate(const std::string& t) { return t == "pin_gate"; }
bool isSubGate(const std::string& t) { return t == "submarine_gate"; }
bool isFanGate(const std::string& t) { return t == "fan_gate"; }
bool isKnownGateType(const std::string& t)
{
    return isPinGate(t) || isSubGate(t) || isFanGate(t);
}

// Sub-gate angle (degrees from entry-face normal).  Slice-1 uses a fixed 30°
// — caller can extend the schema to expose this as input.
constexpr double kSubmarineAngleDeg = 30.0;

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isKnownGateType(in.gate_type)) {
        r.add("DFM-INPUT", "error",
              "mold_gate gate_type '" + in.gate_type +
              "' not in { pin_gate, submarine_gate, fan_gate }");
    }

    if (in.dia_or_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "mold_gate dia_or_width_mm must be > 0");
    } else if (in.dia_or_width_mm < 0.3 || in.dia_or_width_mm > 2.0) {
        r.add("DFM-GATE-SIZE", "error",
              "mold_gate dia_or_width " + std::to_string(in.dia_or_width_mm) +
              " mm outside [0.3, 2.0] mm typical range");
    }

    if (in.length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "mold_gate length_mm must be > 0");
    }

    if (in.dia_or_width_mm > 0.0 && in.length_mm > 0.0 &&
        (isPinGate(in.gate_type) || isSubGate(in.gate_type))) {
        if (in.length_mm < in.dia_or_width_mm * 2.0 - 1e-9) {
            r.add("DFM-GATE-LEN", "warning",
                  "mold_gate length " + std::to_string(in.length_mm) +
                  " < 2 × dia (" + std::to_string(in.dia_or_width_mm * 2.0) +
                  ") — short gates freeze too fast");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mold_gate DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mold_gate: entry_face datum unresolved");

    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("mold_gate: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();
    const gp_Dir inward(-outward.X(), -outward.Y(), -outward.Z());

    // Project position onto entry plane.
    auto projectOntoPlane = [&](double x, double y) {
        const gp_Pnt P0 = entryPlane.Location();
        const gp_Dir n  = entryPlane.Axis().Direction();
        if (std::abs(n.Z()) < 1e-9) return gp_Pnt(x, y, P0.Z());
        const double z = P0.Z() - (n.X() * (x - P0.X()) + n.Y() * (y - P0.Y())) / n.Z();
        return gp_Pnt(x, y, z);
    };
    const gp_Pnt posPnt = projectOntoPlane(in.position_x_mm, in.position_y_mm);

    constexpr double kOverhang = 0.05;

    TopoDS_Shape cutter;

    if (isPinGate(in.gate_type)) {
        const double radius = in.dia_or_width_mm / 2.0;
        // Start outside the entry face, cut inward by length + overhang.
        const gp_Pnt start(
            posPnt.X() - inward.X() * kOverhang,
            posPnt.Y() - inward.Y() * kOverhang,
            posPnt.Z() - inward.Z() * kOverhang);
        const gp_Ax2 ax(start, inward);
        cutter = pr::cylinder(ax, radius, in.length_mm + kOverhang);
    } else if (isSubGate(in.gate_type)) {
        // Submarine gate: same cylinder but axis tilted by kSubmarineAngleDeg
        // from `inward`.  We tilt around an axis perpendicular to inward in
        // the entry plane.  Pick a stable tangent: cross inward with world +X
        // unless inward is parallel to +X, then use +Y.
        const gp_Dir worldRef = (std::abs(inward.X()) > 0.9)
            ? gp_Dir(0.0, 1.0, 0.0)
            : gp_Dir(1.0, 0.0, 0.0);
        gp_Vec rotAxis = gp_Vec(inward).Crossed(gp_Vec(worldRef));
        if (rotAxis.Magnitude() < 1e-9) rotAxis = gp_Vec(0.0, 1.0, 0.0);
        rotAxis.Normalize();
        // Rotate inward by kSubmarineAngleDeg around rotAxis.
        const double a = kSubmarineAngleDeg * M_PI / 180.0;
        gp_Vec inwardVec(inward);
        // Rodrigues' rotation: v_rot = v cos + (k×v) sin + k (k·v)(1-cos)
        const gp_Vec kAxis(rotAxis.X(), rotAxis.Y(), rotAxis.Z());
        const double kDotV = kAxis.Dot(inwardVec);
        gp_Vec kCrossV = kAxis.Crossed(inwardVec);
        gp_Vec rotated = inwardVec * std::cos(a)
                       + kCrossV * std::sin(a)
                       + kAxis * (kDotV * (1.0 - std::cos(a)));
        if (rotated.Magnitude() < 1e-9) rotated = inwardVec;
        rotated.Normalize();
        const gp_Dir tiltedInward(rotated.X(), rotated.Y(), rotated.Z());
        const double radius = in.dia_or_width_mm / 2.0;
        const gp_Pnt start(
            posPnt.X() - tiltedInward.X() * kOverhang,
            posPnt.Y() - tiltedInward.Y() * kOverhang,
            posPnt.Z() - tiltedInward.Z() * kOverhang);
        const gp_Ax2 ax(start, tiltedInward);
        cutter = pr::cylinder(ax, radius, in.length_mm + kOverhang);
    } else {
        // fan_gate: a small flat box.  width = dia_or_width, length =
        // length_mm (along an in-plane direction), thickness = 0.5 × width.
        // Slice-1: in-plane direction = projection of world +X onto entry
        // plane (if degenerate, use +Y).
        gp_Vec worldX(1.0, 0.0, 0.0);
        // Remove component along outward.
        const double dotN = worldX.Dot(gp_Vec(outward));
        gp_Vec inPlaneX = worldX - gp_Vec(outward) * dotN;
        if (inPlaneX.Magnitude() < 1e-3) {
            inPlaneX = gp_Vec(0.0, 1.0, 0.0);
            const double d2 = inPlaneX.Dot(gp_Vec(outward));
            inPlaneX = inPlaneX - gp_Vec(outward) * d2;
        }
        inPlaneX.Normalize();
        const gp_Dir xLoc(inPlaneX.X(), inPlaneX.Y(), inPlaneX.Z());

        const double width = in.dia_or_width_mm;
        const double thickness = std::max(0.1, width * 0.5);

        // Box axis: gp_Ax2(origin, V = inward, Vx = xLoc).  Then local Y =
        // V × Vx = inward × xLoc.  DX along xLoc (length), DY along
        // (inward × xLoc) (width), DZ along inward (thickness).
        gp_Vec yVec = gp_Vec(inward).Crossed(gp_Vec(xLoc));
        if (yVec.Magnitude() < 1e-9) yVec = gp_Vec(0.0, 1.0, 0.0);
        yVec.Normalize();
        const gp_Dir yLoc(yVec.X(), yVec.Y(), yVec.Z());

        // Origin at posPnt - (length/2) xLoc - (width/2) yLoc + overhang × outward
        // (i.e. slightly above the entry plane), so the box starts above the
        // entry face and cuts inward through the surface for a clean cut.
        const gp_Pnt origin(
            posPnt.X() - (in.length_mm / 2.0) * xLoc.X()
                       - (width / 2.0)        * yLoc.X()
                       + kOverhang            * outward.X(),
            posPnt.Y() - (in.length_mm / 2.0) * xLoc.Y()
                       - (width / 2.0)        * yLoc.Y()
                       + kOverhang            * outward.Y(),
            posPnt.Z() - (in.length_mm / 2.0) * xLoc.Z()
                       - (width / 2.0)        * yLoc.Z()
                       + kOverhang            * outward.Z());
        const gp_Ax2 ax(origin, inward, xLoc);
        cutter = pr::box(ax, in.length_mm, width, thickness + kOverhang);
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_kind",  "resolved_id" },
        { "entry_face_id",    *entryId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "gate_type",        in.gate_type },
        { "dia_or_width_mm",  in.dia_or_width_mm },
        { "length_mm",        in.length_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "gate_type",        in.gate_type },
        { "dia_or_width_mm",  in.dia_or_width_mm },
        { "length_mm",        in.length_mm },
        { "axis_dir",         { inward.X(), inward.Y(), inward.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type    = "mold_gate";
    tooling.tool_dia_mm  = in.dia_or_width_mm;
    if (isFanGate(in.gate_type)) {
        tooling.stock_removed_mm3 = in.length_mm * in.dia_or_width_mm
                                  * std::max(0.1, in.dia_or_width_mm * 0.5);
    } else {
        const double r = in.dia_or_width_mm / 2.0;
        tooling.stock_removed_mm3 = M_PI * r * r * in.length_mm;
    }
    tooling.est_cycle_time_s = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::mold_gate applied: type={} size={} length={} faces {}→{}",
                  in.gate_type, in.dia_or_width_mm, in.length_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay first.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",   f.params.value("position_x_mm",   0.0) },
            { "position_y_mm",   f.params.value("position_y_mm",   0.0) },
            { "gate_type",       f.params.value("gate_type",       std::string("pin_gate")) },
            { "dia_or_width_mm", f.params.value("dia_or_width_mm", 0.0) },
            { "length_mm",       f.params.value("length_mm",       0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology heuristic: any cylindrical face with diameter < 2 mm near a
    // boundary edge of a planar face.
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        if (radius > 1.0) continue;       // not gate-sized
        if (radius < 0.10) continue;
        const gp_Pnt loc = cyl.Axis().Location();
        const gp_Dir adir = cyl.Axis().Direction();
        // Use cylinder axis Z component to classify pin (Z) vs submarine (tilted).
        const bool perpendicular = std::abs(std::abs(adir.Z()) - 1.0) < 1e-2;
        const std::string gtype = perpendicular ? "pin_gate" : "submarine_gate";

        json rec = {
            { "position_x_mm",   loc.X() },
            { "position_y_mm",   loc.Y() },
            { "gate_type",       gtype },
            { "dia_or_width_mm", 2.0 * radius },
            { "length_mm",       0.0 },   // unknown without edge analysis
        };
        json matched = { { "cylindrical_face_id", fIdx } };
        out.push_back(RecognizedFeature{ kSkillId, rec, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mold_gate
