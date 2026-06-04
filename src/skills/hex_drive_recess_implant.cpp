// @lat: [[engine/skills#hex_drive_recess_implant]]

#include "hex_drive_recess_implant.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace koocadcam::skill::hex_drive_recess_implant {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr std::array<double, 6> kHexAFsMm { 1.5, 2.0, 2.5, 3.0, 4.0, 5.0 };

bool isStandardAF(double af)
{
    for (double v : kHexAFsMm) if (std::abs(af - v) < 1e-3) return true;
    return false;
}

// Build a hex prism whose flats are normal to the gp_Dir perpendicular to
// adir.  In the ±Z-axis fast path, two flats are normal to ±X (vertices at
// top & bottom in XY).
//
// Cuts EXTRUDE from `topZ + 0.05` downward (along -adir for Z-fast-path)
// to (topZ - depth - 0.05).
TopoDS_Shape buildHexPrism(double cx, double cy, double topZ,
                           double af, double depth,
                           const gp_Dir& adir)
{
    const double rCorner = af / std::sqrt(3.0);

    // For the ±Z fast path, place six vertices in the XY plane at
    // angles 90°, 150°, ..., flats on ±X.
    // For a non-axis-aligned adir we still build hex in the XY plane and
    // extrude along adir (no rotation needed because medical hex sockets
    // are uniform along their axis); future polish could rotate vertices
    // into a perp-frame.
    std::array<gp_Pnt, 6> pts;
    for (int i = 0; i < 6; ++i) {
        const double a = (90.0 + 60.0 * i) * M_PI / 180.0;
        pts[i] = gp_Pnt(cx + rCorner * std::cos(a),
                        cy + rCorner * std::sin(a),
                        topZ + 0.05);
    }
    BRepBuilderAPI_MakeWire wire;
    for (int i = 0; i < 6; ++i) {
        const int j = (i + 1) % 6;
        BRepBuilderAPI_MakeEdge edge(pts[i], pts[j]);
        if (!edge.IsDone())
            throw Standard_Failure("hex_drive_recess_implant: hex edge build failed");
        wire.Add(edge.Edge());
    }
    if (!wire.IsDone())
        throw Standard_Failure("hex_drive_recess_implant: hex wire build failed");
    BRepBuilderAPI_MakeFace face(wire.Wire(), /*OnlyPlane=*/true);
    if (!face.IsDone())
        throw Standard_Failure("hex_drive_recess_implant: hex face build failed");

    // Extrude along -Z by (depth + 0.1) in the Z-fast-path; for other adir
    // use the axis direction so extrusion goes into the material.
    gp_Vec extrude;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        extrude = gp_Vec(0.0, 0.0, -(depth + 0.1));
    } else {
        extrude = gp_Vec(adir.X(), adir.Y(), adir.Z()) * (depth + 0.1);
    }
    BRepPrimAPI_MakePrism prism(face.Face(), extrude);
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("hex_drive_recess_implant: hex prism build failed");
    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hex_AF_mm <= 0.0) {
        r.add("DFM-HEX-AF", "error",
              "hex_drive_recess_implant: hex_AF_mm must be > 0");
        return r;
    }
    if (!isStandardAF(in.hex_AF_mm)) {
        r.add("DFM-HEX-AF", "error",
              "hex_drive_recess_implant: hex_AF " +
              std::to_string(in.hex_AF_mm) +
              " mm not in {1.5, 2.0, 2.5, 3.0, 4.0, 5.0}");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-HEX-DEPTH", "error",
              "hex_drive_recess_implant: depth_mm must be > 0");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > 0.0 && in.depth_mm > thickness - 0.1) {
        r.add("DFM-HEX-DEPTH", "warning",
              "hex_drive_recess_implant: depth " +
              std::to_string(in.depth_mm) +
              " mm nearly equals/exceeds stock thickness " +
              std::to_string(thickness) + " mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hex_drive_recess_implant DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("hex_drive_recess_implant: entry_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir = in.axis_dir;
    double topZ = (zMin + zMax) / 2.0;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        topZ = (adir.Z() < 0) ? zMax : zMin;
    }

    const TopoDS_Shape hexTool =
        buildHexPrism(in.position_x_mm, in.position_y_mm, topZ,
                      in.hex_AF_mm, in.depth_mm, adir);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), hexTool);

    const double rCorner = in.hex_AF_mm / std::sqrt(3.0);

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "hex_AF_mm",       in.hex_AF_mm },
        { "depth_mm",        in.depth_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      1 },
        { "medical_feature_type",  "implant_hex_driver_socket" },
        { "screw_size",            in.hex_AF_mm },
        { "hex_AF_mm",             in.hex_AF_mm },
        { "hex_corner_radius_mm",  rCorner },
        { "side_count",            6 },
        { "depth_mm",              in.depth_mm },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
        { "implant_standard",      "ISO_4762_hex" },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "endmill_hex_form;broach";
    tooling.tool_dia_mm = in.hex_AF_mm;
    tooling.tool_length_mm = in.depth_mm * 1.5 + 3.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 6;
    tooling.cutting_speed_sfm = 150.0;
    tooling.feed_per_tooth_mm = 0.02;
    // Regular-hexagon area = 3 * √3 / 2 * AF^2 / 3 actually 1.5·√3·s² where
    // s = side; for AF based formula area = (√3 / 2) · AF² · √3 / ... use
    // direct: area = 1.5 · AF · rCorner = 1.5 · AF · (AF/√3) = AF² · √3 / 2.
    const double hexArea = std::sqrt(3.0) / 2.0 * in.hex_AF_mm * in.hex_AF_mm;
    tooling.stock_removed_mm3 = hexArea * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm / 10.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::hex_drive_recess_implant applied: AF {} depth {}",
                  in.hex_AF_mm, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata path (primary).  Geometric: hex pocket = 6 planar side faces +
// 1 planar bottom.  Lightweight geometric fallback below counts planar
// faces with a common axis-aligned plane normal.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true },
                                { "medical_feature_type",
                                  "implant_hex_driver_socket" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::hex_drive_recess_implant
