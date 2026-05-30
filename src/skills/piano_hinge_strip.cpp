// @lat: [[engine/skills#piano_hinge_strip]]

#include "piano_hinge_strip.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::piano_hinge_strip {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.knuckle_count < 2) {
        r.add("DFM-PIANO-KNUCKLES", "error",
              "piano_hinge_strip: knuckle_count " +
              std::to_string(in.knuckle_count) +
              " < 2 (need at least 2 knuckles to be a hinge)");
    }
    if (in.pin_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "piano_hinge_strip: pin_dia " + std::to_string(in.pin_dia_mm) +
              " mm < 0.8 mm");
    }
    if (in.mount_hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "piano_hinge_strip: mount_hole_dia " +
              std::to_string(in.mount_hole_dia_mm) + " mm < 0.8 mm");
    }
    if (in.strip_length_mm <= 0.0 || in.strip_width_mm <= 0.0
        || in.strip_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "piano_hinge_strip: strip dimensions must be > 0");
    }
    if (in.knuckle_pitch_mm > 0.0 && in.pin_dia_mm > 0.0
        && in.knuckle_pitch_mm <= 1.5 * in.pin_dia_mm) {
        r.add("DFM-PIANO-PITCH", "error",
              "piano_hinge_strip: knuckle_pitch " +
              std::to_string(in.knuckle_pitch_mm) +
              " mm <= 1.5 × pin_dia (" +
              std::to_string(1.5 * in.pin_dia_mm) +
              " mm) — knuckle webs too thin");
    }
    if (in.strip_thickness_mm > 0.0 && in.pin_dia_mm > 0.0
        && in.strip_thickness_mm <= in.pin_dia_mm + 0.6) {
        r.add("DFM-PIANO-THICK", "error",
              "piano_hinge_strip: strip_thickness " +
              std::to_string(in.strip_thickness_mm) +
              " mm too thin for pin (need > pin_dia + 0.6 mm)");
    }
    // Total knuckle span must fit within strip length
    const double span = (in.knuckle_count - 1) * in.knuckle_pitch_mm
                        + in.knuckle_slot_width_mm;
    if (span > in.strip_length_mm) {
        r.add("DFM-PIANO-FIT", "error",
              "piano_hinge_strip: knuckle span " + std::to_string(span) +
              " mm exceeds strip_length " +
              std::to_string(in.strip_length_mm) + " mm");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "piano_hinge_strip DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("piano_hinge_strip: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // For slice-1 we support pin_axis_dir = +X.  Strip extends along +X for
    // strip_length, along +Y for strip_width, and goes -Z into material.
    if (std::abs(in.pin_axis_dir.X() - 1.0) > 1e-3) {
        throw SkillError("piano_hinge_strip: only pin_axis_dir = +X supported in slice 1");
    }

    const double zTop    = zMax;
    const double slotDepth =
        (in.knuckle_slot_depth_mm > 0.0)
            ? in.knuckle_slot_depth_mm
            : in.strip_thickness_mm * 0.5;

    // ── Sub-feature 1: knuckle slots fused into a single compound cutter
    std::vector<TopoDS_Shape> slotCutters;
    slotCutters.reserve(in.knuckle_count);
    for (int k = 0; k < in.knuckle_count; ++k) {
        const double cx = in.origin_x_mm + k * in.knuckle_pitch_mm
                          + in.knuckle_slot_width_mm * 0.5;
        // Slot box centered on cx along X, full strip width along Y, depth -Z
        const gp_Pnt origin(
            cx - in.knuckle_slot_width_mm / 2.0,
            in.origin_y_mm,
            zTop - slotDepth);
        const gp_Ax2 ax(origin, gp::DZ());
        const TopoDS_Shape slot = pr::box(
            ax,
            in.knuckle_slot_width_mm,
            in.strip_width_mm,
            slotDepth + kOverhang);
        slotCutters.push_back(slot);
    }

    // ── Sub-feature 2: pin bore (cylinder along +X) ─────────────────────
    // Pin axis sits at the strip's Y mid-line, at depth pin_dia/2 + clearance
    // below top.  We center it at Y = origin_y + strip_width/2,
    // Z = zTop - strip_thickness/2.
    const double pinY = in.origin_y_mm + in.strip_width_mm / 2.0;
    const double pinZ = zTop - in.strip_thickness_mm / 2.0;
    // Pin axis starts a bit before strip start and extends a bit past end
    // so the Boolean cut is clean.
    const gp_Pnt pinStart(in.origin_x_mm - kOverhang, pinY, pinZ);
    const gp_Ax2 pinAx(pinStart, gp_Dir(1.0, 0.0, 0.0));
    const TopoDS_Shape pinBore = pr::cylinder(
        pinAx, in.pin_dia_mm / 2.0,
        in.strip_length_mm + 2.0 * kOverhang);

    // ── Sub-feature 3: 2 × N mounting holes ─────────────────────────────
    // Two rows of N holes: one at y = origin_y + inset (front edge), one at
    // y = origin_y + strip_width - inset (back edge).  X positions sit
    // between knuckles (at midpoints of the pitch intervals).
    std::vector<TopoDS_Shape> mountCutters;
    mountCutters.reserve(2 * in.knuckle_count);
    for (int k = 0; k < in.knuckle_count; ++k) {
        const double cx = in.origin_x_mm + k * in.knuckle_pitch_mm
                          + in.knuckle_slot_width_mm * 0.5;
        for (int side = 0; side < 2; ++side) {
            const double hy = (side == 0)
                ? in.origin_y_mm + in.mount_hole_inset_mm
                : in.origin_y_mm + in.strip_width_mm - in.mount_hole_inset_mm;
            const gp_Pnt hStart(cx, hy, zTop + kOverhang);
            const gp_Ax2 hAx(hStart, gp_Dir(0.0, 0.0, -1.0));
            const TopoDS_Shape h = pr::cylinder(
                hAx, in.mount_hole_dia_mm / 2.0,
                in.strip_thickness_mm + 2.0 * kOverhang);
            mountCutters.push_back(h);
        }
    }

    // Fuse all the slot+pin+mount cutters and apply one final cut.
    std::vector<TopoDS_Shape> all;
    all.reserve(slotCutters.size() + 1 + mountCutters.size());
    for (const auto& s : slotCutters) all.push_back(s);
    all.push_back(pinBore);
    for (const auto& m : mountCutters) all.push_back(m);
    const TopoDS_Shape fusedAll = pr::fuseMany(all.front(),
        std::vector<TopoDS_Shape>(all.begin() + 1, all.end()));
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedAll);

    json params = {
        { "entry_face_id",            *entryId },
        { "origin_x_mm",              in.origin_x_mm },
        { "origin_y_mm",              in.origin_y_mm },
        { "pin_axis_dir",             { in.pin_axis_dir.X(), in.pin_axis_dir.Y(), in.pin_axis_dir.Z() } },
        { "strip_length_mm",          in.strip_length_mm },
        { "strip_width_mm",           in.strip_width_mm },
        { "strip_thickness_mm",       in.strip_thickness_mm },
        { "knuckle_count",            in.knuckle_count },
        { "knuckle_pitch_mm",         in.knuckle_pitch_mm },
        { "knuckle_slot_width_mm",    in.knuckle_slot_width_mm },
        { "knuckle_slot_depth_mm",    slotDepth },
        { "pin_dia_mm",               in.pin_dia_mm },
        { "mount_hole_dia_mm",        in.mount_hole_dia_mm },
        { "mount_hole_inset_mm",      in.mount_hole_inset_mm },
    };
    const int subfeatureCount = in.knuckle_count + 1 + 2 * in.knuckle_count;
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "subfeature_count", subfeatureCount },
        { "knuckle_count",    in.knuckle_count },
        { "mount_hole_count", 2 * in.knuckle_count },
        { "pin_dia_mm",       in.pin_dia_mm },
        { "strip_length_mm",  in.strip_length_mm },
        { "pin_axis_dir",     { in.pin_axis_dir.X(), in.pin_axis_dir.Y(), in.pin_axis_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::max(in.knuckle_slot_width_mm, in.pin_dia_mm);
    tooling.tool_length_mm    = std::max(slotDepth, in.strip_width_mm) + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.03;
    const double slotVol  = in.knuckle_slot_width_mm * in.strip_width_mm
                            * slotDepth * in.knuckle_count;
    const double pinVol   = M_PI * std::pow(in.pin_dia_mm / 2.0, 2.0)
                            * in.strip_length_mm;
    const double mountVol = M_PI * std::pow(in.mount_hole_dia_mm / 2.0, 2.0)
                            * in.strip_thickness_mm * 2 * in.knuckle_count;
    tooling.stock_removed_mm3 = slotVol + pinVol + mountVol;
    tooling.est_cycle_time_s  = std::max(10.0,
        in.knuckle_count * 1.5 + in.strip_length_mm * 0.1
        + 2.0 * in.knuckle_count * 0.5);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.knuckle_slot_width_mm },
              { "pass_count",  in.knuckle_count },
              { "purpose",     "knuckle slots (ANSI BHMA A156.1)" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.pin_dia_mm },
              { "depth_mm",    in.strip_length_mm },
              { "purpose",     "through pin bore" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.mount_hole_dia_mm },
              { "pass_count",  2 * in.knuckle_count },
              { "purpose",     "mounting screw holes (DIN 7951)" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::piano_hinge_strip applied: knuckles={} pin_dia={} len={} faces {}→{}",
                  in.knuckle_count, in.pin_dia_mm, in.strip_length_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// PRIMARY (geometric):
//   1. Find the longest X-axis cylindrical face — that is the pin bore.
//      Its radius gives pin_dia_mm; its X-extent gives strip_length_mm.
//   2. Cluster Z-axis mounting-hole edges by X coordinate; each X cluster
//      corresponds to one knuckle row (~2 holes, one per side).
//   3. knuckle_count = number of X clusters; knuckle_pitch ≈ mean spacing.
//
// FALLBACK (metadata replay): scan wp.features() for a matching skill_id.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // FALLBACK FIRST: metadata replay (highest confidence) ────────────────
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "via", "metadata_replay" } }
        });
    }
    if (!out.empty()) return out;

    // PRIMARY: geometric heuristic ─────────────────────────────────────
    int xCylCount = 0;
    double pinDiaGuess = 0.0;
    gp_Pnt pinCenter(0, 0, 0);
    bool   pinFound = false;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double stripLen = xMax - xMin;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder c = surf.Cylinder();
        const gp_Dir axDir = c.Axis().Direction();
        if (std::abs(std::abs(axDir.X()) - 1.0) < 1e-2) {
            xCylCount++;
            if (c.Radius() > pinDiaGuess / 2.0) {
                pinDiaGuess = 2.0 * c.Radius();
                pinCenter   = c.Axis().Location();
                pinFound    = true;
            }
        }
    }

    // Collect small Z-axis circular edge centers (top X coordinate of each
    // mounting hole).  Cluster by X coordinate at ±2 mm tolerance to merge
    // the top and bottom circles of the same hole, and to merge the two
    // mounting holes (left/right side) belonging to the same knuckle row.
    std::vector<double> mountX;
    for (int e = 0; e < wp.edgeCount(); ++e) {
        if (!wp.isEdgeCircle(e)) continue;
        BRepAdaptor_Curve crv(wp.edge(e));
        const gp_Circ ce = crv.Circle();
        const gp_Dir d = ce.Axis().Direction();
        if (std::abs(std::abs(d.Z()) - 1.0) > 1e-2) continue;
        if (ce.Radius() > 4.0) continue;  // mounting holes are small
        mountX.push_back(ce.Location().X());
    }
    std::sort(mountX.begin(), mountX.end());

    // Cluster by X with 3 mm tolerance — knuckle slots are wider than
    // mount-hole pitch within a single knuckle row.
    std::vector<double> knuckleX;
    for (double x : mountX) {
        if (knuckleX.empty() || std::abs(x - knuckleX.back()) > 3.0) {
            knuckleX.push_back(x);
        }
    }

    const int knuckleGuess = std::max(2, static_cast<int>(knuckleX.size()));
    double pitchGuess = 25.0;
    if (knuckleX.size() >= 2) {
        double sumSpacing = 0.0;
        for (size_t k = 1; k < knuckleX.size(); ++k) {
            sumSpacing += knuckleX[k] - knuckleX[k - 1];
        }
        pitchGuess = sumSpacing / static_cast<double>(knuckleX.size() - 1);
    }

    if (pinFound && xCylCount >= 1 && mountX.size() >= 4) {
        // Confidence: base 0.7, +0.05 if pin axis spans the full strip,
        // +0.05 if at least 2 knuckle clusters detected.
        double confidence = 0.70;
        if (pinDiaGuess > 0.5 && stripLen / pinDiaGuess > 5.0) confidence += 0.05;
        if (knuckleX.size() >= 2) confidence += 0.05;

        json recovered = {
            { "origin_x_mm",            xMin },
            { "origin_y_mm",            yMin },
            { "pin_axis_dir",           { 1.0, 0.0, 0.0 } },
            { "strip_length_mm",        stripLen },
            { "strip_width_mm",         yMax - yMin },
            { "strip_thickness_mm",     zMax - zMin },
            { "knuckle_count",          knuckleGuess },
            { "knuckle_pitch_mm",       pitchGuess },
            { "pin_dia_mm",             pinDiaGuess },
        };
        json matched = {
            { "source",                "geometric_primary" },
            { "x_axis_cyl_face_count", xCylCount },
            { "mount_hole_edge_count", static_cast<int>(mountX.size()) },
            { "knuckle_cluster_count", static_cast<int>(knuckleX.size()) },
            { "pin_center",            { pinCenter.X(), pinCenter.Y(), pinCenter.Z() } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, confidence, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::piano_hinge_strip
