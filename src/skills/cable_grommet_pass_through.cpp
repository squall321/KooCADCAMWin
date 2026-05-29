// @lat: [[engine/skills#cable_grommet_pass_through]]

#include "cable_grommet_pass_through.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::cable_grommet_pass_through {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Grommet spec table ───────────────────────────────────────────────────
//
// Dimensions follow ISO 16916 (annular rubber grommets — slot-on).  The
// retention groove is what the inside rib of the grommet snaps into.

namespace {

struct GrommetSpec {
    const char* size;
    double bore_dia_mm;
    double groove_depth_mm;     // radial depth of the annular groove
    double groove_width_mm;     // axial width of the groove
    double groove_z_offset_mm;  // distance from entry face to groove center
    double chamfer_mm;
};

constexpr std::array<GrommetSpec, 5> kGrommetTable {{
    { "GR-6",   6.0, 0.4, 0.6, 1.0, 0.3 },
    { "GR-8",   8.0, 0.5, 0.7, 1.2, 0.4 },
    { "GR-10", 10.0, 0.6, 0.8, 1.5, 0.5 },
    { "GR-12", 12.0, 0.7, 1.0, 1.8, 0.6 },
    { "GR-16", 16.0, 0.9, 1.2, 2.2, 0.8 },
}};

const GrommetSpec* findSpec(const std::string& size)
{
    for (const auto& g : kGrommetTable) {
        if (size == g.size) return &g;
    }
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const GrommetSpec* sp = findSpec(in.grommet_size);
    if (!sp) {
        r.add("DFM-CONN-SIZE", "error",
              "cable_grommet_pass_through: unknown grommet_size '" +
              in.grommet_size +
              "' (expected GR-6/GR-8/GR-10/GR-12/GR-16)");
        return r;
    }

    if (sp->bore_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "cable_grommet_pass_through: bore Ø " +
              std::to_string(sp->bore_dia_mm) + " < 0.8 mm");
    }
    if (sp->groove_width_mm < 0.5) {
        r.add("DFM-GROOVE-WIDTH", "error",
              "cable_grommet_pass_through: groove width " +
              std::to_string(sp->groove_width_mm) +
              " < 0.5 mm (form-grind clearance)");
    }

    // Plate thickness check: must accommodate the entry chamfer + the
    // groove location + groove width + 1 mm of solid material below.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    const double minThick =
        sp->chamfer_mm + sp->groove_z_offset_mm + sp->groove_width_mm / 2.0 + 1.0;
    if (thickness > 0.0 && thickness < minThick) {
        r.add("DFM-PLATE-THICKNESS", "error",
              "cable_grommet_pass_through: plate thickness " +
              std::to_string(thickness) +
              " < min " + std::to_string(minThick) + " mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-cuts (concentric, fused):
//   1. THROUGH BORE     — straight cylinder of bore_dia × full thickness
//   2. RETENTION GROOVE — outer annular cylinder (bore_dia + 2·groove_depth)
//                         of width groove_width_mm, centered at z = entry − groove_z_offset
//   3. ENTRY CHAMFER    — cone frustum at the entry edge

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cable_grommet_pass_through DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const GrommetSpec* sp = findSpec(in.grommet_size);
    const double boreR    = sp->bore_dia_mm / 2.0;
    const double grooveR  = boreR + sp->groove_depth_mm;
    const double chamferR = boreR + sp->chamfer_mm;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    const double kOverhang = 0.05;

    // Entry face: zMax for axis -Z (default), else zMin.
    const double entryZ = (in.axis_dir.Z() < 0) ? zMax : zMin;
    const double signZ  = (in.axis_dir.Z() < 0) ? -1.0 : 1.0;

    // ── Sub-cut 1: through bore ──────────────────────────────────────────
    const gp_Ax2 axBore(
        gp_Pnt(in.position_x_mm, in.position_y_mm, entryZ + signZ * kOverhang),
        in.axis_dir);
    const TopoDS_Shape bore = pr::cylinder(
        axBore, boreR, thickness + 2.0 * kOverhang);

    // ── Sub-cut 2: retention groove (annular ring) ───────────────────────
    // Position groove center at entryZ + signZ * groove_z_offset_mm.
    const double grooveCenterZ = entryZ + signZ * sp->groove_z_offset_mm;
    const double grooveStartZ  = grooveCenterZ - sp->groove_width_mm / 2.0;
    const gp_Ax2 axGroove(
        gp_Pnt(in.position_x_mm, in.position_y_mm, grooveStartZ),
        gp::DZ());
    const TopoDS_Shape groove = pr::annularRing(
        axGroove, grooveR, boreR * 0.99, sp->groove_width_mm);

    // ── Sub-cut 3: entry chamfer (cone frustum) ──────────────────────────
    // Top radius = chamferR, bottom radius = boreR, height = chamfer_mm.
    const gp_Ax2 axChamfer(
        gp_Pnt(in.position_x_mm, in.position_y_mm,
               entryZ - signZ * sp->chamfer_mm),
        gp::DZ());
    const TopoDS_Shape chamfer = pr::coneFrustum(
        axChamfer, chamferR, boreR, sp->chamfer_mm);

    // FUSE the three concentric / overlapping cutters into one.
    TopoDS_Shape fusedCutter = pr::fuse(bore, groove);
    fusedCutter = pr::fuse(fusedCutter, chamfer);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "axis_dir",      { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "grommet_size",  in.grommet_size },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },     // bore + groove + chamfer
        { "grommet_size",        in.grommet_size },
        { "bore_dia_mm",         sp->bore_dia_mm },
        { "groove_depth_mm",     sp->groove_depth_mm },
        { "groove_width_mm",     sp->groove_width_mm },
        { "groove_z_offset_mm",  sp->groove_z_offset_mm },
        { "chamfer_mm",          sp->chamfer_mm },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "position_x_mm",       in.position_x_mm },
        { "position_y_mm",       in.position_y_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;endmill;chamfer";
    tooling.tool_dia_mm      = sp->bore_dia_mm;
    tooling.tool_length_mm   = thickness * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 =
        M_PI * boreR * boreR * thickness +
        M_PI * (grooveR * grooveR - boreR * boreR) * sp->groove_width_mm;
    tooling.est_cycle_time_s = std::max(2.0, thickness * 0.5);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },       { "tool_dia_mm", sp->bore_dia_mm } },
            { { "tool_type", "groove_form" }, { "depth_mm", sp->groove_depth_mm },
              { "width_mm", sp->groove_width_mm } },
            { { "tool_type", "chamfer" },     { "depth_mm", sp->chamfer_mm } },
        } },
        { "grommet_standard", "ISO_16916" },
        { "grommet_size",     in.grommet_size },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cable_grommet_pass_through applied: {} at ({},{})",
                  in.grommet_size, in.position_x_mm, in.position_y_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: find a cylindrical face whose radius matches one
// of the grommet bore Ø in the spec table.  Returns metadata replay
// FIRST when available.

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
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric heuristic.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        const double dia = 2.0 * cyl.Radius();
        for (const auto& g : kGrommetTable) {
            if (std::abs(dia - g.bore_dia_mm) < 0.1) {
                const gp_Pnt loc = cyl.Axis().Location();
                json rp = {
                    { "axis_dir",      json::array({ 0.0, 0.0, -1.0 }) },
                    { "position_x_mm", loc.X() },
                    { "position_y_mm", loc.Y() },
                    { "grommet_size",  g.size },
                };
                RecognizedFeature rf;
                rf.skill_id         = kSkillId;
                rf.recovered_params = rp;
                rf.confidence       = 0.65;
                rf.matched_geometry = {
                    { "face_id",      i },
                    { "matched_size", g.size },
                    { "source",       "geometric" },
                };
                out.push_back(rf);
                break;
            }
        }
    }
    return out;
}

}  // namespace koocadcam::skill::cable_grommet_pass_through
