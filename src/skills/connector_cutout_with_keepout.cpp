// @lat: [[engine/skills#connector_cutout_with_keepout]]

#include "connector_cutout_with_keepout.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::connector_cutout_with_keepout {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Connector spec table ─────────────────────────────────────────────────
//
// Body dimensions follow the published mating-face specs:
//   D-sub-9   per MIL-DTL-24308 / IEC 60807
//   USB-A     per USB 2.0 spec (Section 6.5.1)
//   USB-C     per USB-IF Type-C 2.0 spec
//   RJ45      per IEC 60603-7 (Cat-5 / Cat-6 receptacle)
//   HDMI      per HDMI Licensing 1.4 (Type A)
//
// EMI keepout per IPC-2221 §6.3 (≥ 1 mm border around metallic shell).

namespace {

struct ConnSpec {
    const char* name;
    double body_w_mm;
    double body_h_mm;
    double corner_radius_mm;
    int    retention_notch_count;
    double retention_notch_w_mm;
    double retention_notch_d_mm;
    double chamfer_mm;
    double keepout_w_mm;
    double keepout_h_mm;
};

constexpr std::array<ConnSpec, 5> kConnTable {{
    // D-sub-9: 16.92 × 8.36, no notch (uses screw lugs), chamfer 0.3
    { "D-sub-9", 16.92,  8.36, 0.3, 0,  0.0,  0.0, 0.3, 24.0, 12.0 },
    // USB-A: 12.00 × 4.50, two side notches, chamfer 0.2
    { "USB-A",   12.00,  4.50, 0.2, 2,  1.50, 0.40, 0.2, 16.0,  8.0 },
    // USB-C: 8.94 × 2.56, two micro notches, chamfer 0.15
    { "USB-C",    8.94,  2.56, 0.15,2,  0.80, 0.30, 0.15,12.0,  6.0 },
    // RJ45: 12.70 × 14.50, one bottom retention notch, chamfer 0.3
    { "RJ45",    12.70, 14.50, 0.4, 1,  3.00, 0.50, 0.3, 16.0, 18.0 },
    // HDMI Type A: 14.00 × 4.55, two side notches, chamfer 0.2
    { "HDMI",    14.00,  4.55, 0.25,2,  2.00, 0.40, 0.2, 18.0,  8.0 },
}};

const ConnSpec* findSpec(const std::string& name)
{
    for (const auto& c : kConnTable) {
        if (name == c.name) return &c;
    }
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const ConnSpec* sp = findSpec(in.connector_type);
    if (!sp) {
        r.add("DFM-CONN-SIZE", "error",
              "connector_cutout_with_keepout: unknown connector_type '" +
              in.connector_type +
              "' (expected D-sub-9/USB-A/USB-C/RJ45/HDMI)");
        return r;
    }

    if (sp->body_w_mm < 0.8 || sp->body_h_mm < 0.8) {
        r.add("DFM-002", "error",
              "connector_cutout_with_keepout: connector body dim < 0.8 mm");
    }

    const double chamfer = (in.chamfer_override_mm > 0.0)
        ? in.chamfer_override_mm : sp->chamfer_mm;

    // Plate must be at least chamfer + 0.5 mm thick to host the cutout.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > 0.0 && thickness < chamfer + 0.5) {
        r.add("DFM-PLATE-THICKNESS", "error",
              "connector_cutout_with_keepout: panel thickness " +
              std::to_string(thickness) + " < chamfer + 0.5 mm");
    }

    // Connector must fit inside the panel bounding box plus 1 mm keepout
    // border (IPC-2221).
    if (in.position_x_mm - sp->body_w_mm / 2.0 < xMin + 1.0 ||
        in.position_x_mm + sp->body_w_mm / 2.0 > xMax - 1.0 ||
        in.position_y_mm - sp->body_h_mm / 2.0 < yMin + 1.0 ||
        in.position_y_mm + sp->body_h_mm / 2.0 > yMax - 1.0) {
        r.add("DFM-CONN-CLEARANCE", "error",
              "connector_cutout_with_keepout: connector body falls outside "
              "1-mm keepout border on entry face");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "connector_cutout_with_keepout DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ConnSpec* sp = findSpec(in.connector_type);
    const double chamfer = (in.chamfer_override_mm > 0.0)
        ? in.chamfer_override_mm : sp->chamfer_mm;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    const double kOverhang = 0.05;

    // CONTEXT-AWARE: select entry Z based on axis direction.  For panel
    // hosts (axis -Z), entry is at zMax; for inverted, zMin.
    const double entryZ = (in.axis_dir.Z() < 0) ? zMax : zMin;
    const double cutLen = thickness + 2.0 * kOverhang;

    // ── Sub-cut 1: rounded-rectangle through-pocket ──────────────────────
    const gp_Pnt bottomCenter(
        in.position_x_mm, in.position_y_mm,
        (in.axis_dir.Z() < 0) ? zMin - kOverhang : zMin - kOverhang);
    const TopoDS_Shape pocket = pr::roundedRectPocketTool(
        bottomCenter, sp->body_w_mm, sp->body_h_mm, cutLen,
        sp->corner_radius_mm);

    // ── Sub-cut 2: entry-edge chamfer (cone frustum) ─────────────────────
    // Slightly larger rounded-rect that extends just chamfer_mm into the
    // surface — modelled as a rectangular frustum approximation via a
    // larger box.  For simplicity model as a rectangular ring cut.
    TopoDS_Shape chamferCut;
    bool hasChamfer = chamfer > 1e-6;
    if (hasChamfer) {
        const gp_Pnt cbCenter(in.position_x_mm, in.position_y_mm,
                              entryZ - chamfer);
        chamferCut = pr::roundedRectPocketTool(
            cbCenter,
            sp->body_w_mm + 2.0 * chamfer,
            sp->body_h_mm + 2.0 * chamfer,
            chamfer + kOverhang,
            sp->corner_radius_mm + chamfer);
    }

    // ── Sub-cut 3: retention notches (small rectangular pockets) ─────────
    std::vector<TopoDS_Shape> notches;
    if (sp->retention_notch_count > 0 && sp->retention_notch_w_mm > 0.0) {
        // Notches on the long sides (±Y of the body rectangle midline).
        const double notchY[2] = {
            in.position_y_mm + sp->body_h_mm / 2.0,
            in.position_y_mm - sp->body_h_mm / 2.0 - sp->retention_notch_d_mm,
        };
        const int sideCount = std::min(2, sp->retention_notch_count);
        const int perSide = sp->retention_notch_count / sideCount;
        for (int side = 0; side < sideCount; ++side) {
            for (int n = 0; n < perSide; ++n) {
                const double xOffset = (perSide == 1) ? 0.0
                    : ((n - (perSide - 1) / 2.0) * (sp->body_w_mm / perSide));
                const gp_Pnt nCenter(
                    in.position_x_mm + xOffset
                        - sp->retention_notch_w_mm / 2.0,
                    notchY[side],
                    zMin - kOverhang);
                const gp_Ax2 axN(nCenter, gp::DZ());
                const TopoDS_Shape n_shape = pr::box(
                    axN,
                    sp->retention_notch_w_mm,
                    sp->retention_notch_d_mm,
                    cutLen);
                notches.push_back(n_shape);
            }
        }
    }

    // FUSE pocket + chamfer + notches → one cutter; one BRepAlgoAPI_Cut.
    TopoDS_Shape fusedCutter = pocket;
    if (hasChamfer) fusedCutter = pr::fuse(fusedCutter, chamferCut);
    for (const auto& n_shape : notches) {
        fusedCutter = pr::fuse(fusedCutter, n_shape);
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ────────────────────────────────────────────────────────
    const int subCount = 1 + (hasChamfer ? 1 : 0) +
                         static_cast<int>(notches.size());

    json keepout = {
        { "w_mm", sp->keepout_w_mm },
        { "h_mm", sp->keepout_h_mm },
        { "center_x_mm", in.position_x_mm },
        { "center_y_mm", in.position_y_mm },
        { "emi_standard", "IPC-2221" },
    };

    json params = {
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "connector_type",      in.connector_type },
        { "position_x_mm",       in.position_x_mm },
        { "position_y_mm",       in.position_y_mm },
        { "chamfer_override_mm", in.chamfer_override_mm },
        { "chamfer_applied_mm",  chamfer },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     subCount },
        { "connector_type",       in.connector_type },
        { "body_w_mm",            sp->body_w_mm },
        { "body_h_mm",            sp->body_h_mm },
        { "retention_notch_count", sp->retention_notch_count },
        { "emi_keepout",          keepout },
        { "axis_dir",             { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "endmill";
    tooling.tool_dia_mm      = sp->corner_radius_mm * 2.0;
    tooling.tool_length_mm   = thickness * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 =
        sp->body_w_mm * sp->body_h_mm * thickness;
    tooling.est_cycle_time_s = std::max(2.0, thickness * 0.5);
    tooling.extra = {
        { "connector_type",       in.connector_type },
        { "emi_keepout",          keepout },
        { "retention_notch_count", sp->retention_notch_count },
        { "chamfer_mm",           chamfer },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::connector_cutout_with_keepout applied: {} at "
                  "({},{}) chamfer={}",
                  in.connector_type, in.position_x_mm, in.position_y_mm,
                  chamfer);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay is the PRIMARY signal here — connector pocket sizes
// overlap between specs.  Geometric fallback: look for any
// rectangular through-pocket whose footprint matches one of the spec
// W × H boxes within tolerance.

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

    // Geometric fallback: scan planar faces with normal in the +/- Z
    // direction whose dims are close to one of the spec rectangles.
    // We approximate by counting how many wp faces are planar with the
    // same Z normal — heuristic only.
    int planar_norm_count = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(std::abs(n.Z()) - 1.0) < 0.05) planar_norm_count++;
    }
    if (planar_norm_count < 4) return out;

    // Without further geometric measurement, return a low-confidence
    // generic USB-A guess.
    json rp = {
        { "axis_dir",       json::array({ 0.0, 0.0, -1.0 }) },
        { "connector_type", "USB-A" },
        { "position_x_mm",  0.0 },
        { "position_y_mm",  0.0 },
        { "chamfer_override_mm", -1.0 },
        { "chamfer_applied_mm",  0.2 },
    };
    RecognizedFeature rf;
    rf.skill_id         = kSkillId;
    rf.recovered_params = rp;
    rf.confidence       = 0.35;
    rf.matched_geometry = { { "source", "geometric_heuristic" } };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::connector_cutout_with_keepout
