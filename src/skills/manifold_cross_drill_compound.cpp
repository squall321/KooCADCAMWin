// @lat: [[engine/skills#manifold_cross_drill_compound]]

#include "manifold_cross_drill_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::manifold_cross_drill_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Plug counterbore seat diameter lookup (rough — matches NPT/BSPT fitting OD
// for a given metric thread call-out; coarse for slice-1, but plausible).
struct PlugEntry { const char* size; double seat_dia_mm; double min_drill_dia_mm; };
constexpr std::array<PlugEntry, 4> kPlugTable {{
    { "M6",   8.0, 3.5 },
    { "M8",  10.5, 5.0 },
    { "M10", 13.0, 6.5 },
    { "M12", 15.5, 8.0 },
}};

const PlugEntry* lookupPlug(const std::string& s)
{
    for (const auto& e : kPlugTable) if (s == e.size) return &e;
    return nullptr;
}

bool perpendicular(const gp_Dir& a, const gp_Dir& b)
{
    const double dot = a.X() * b.X() + a.Y() * b.Y() + a.Z() * b.Z();
    return std::abs(dot) < 1e-3;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.drill_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "manifold_cross_drill_compound: drill_dia_mm must be > 0");
        return r;
    }
    if (in.drill_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "manifold_cross_drill_compound: drill_dia " +
              std::to_string(in.drill_dia_mm) + " mm < min 0.8 mm");
    }
    if (in.drill_dia_mm < 2.0) {
        r.add("DFM-MANIFOLD-CROSS", "error",
              "manifold_cross_drill_compound: drill_dia " +
              std::to_string(in.drill_dia_mm) +
              " mm < 2 mm — cross intersection chamber too small to flow");
    }
    if (in.plug_seat_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "manifold_cross_drill_compound: plug_seat_depth_mm must be > 0");
    }

    if (!perpendicular(in.drill1_axis, in.drill2_axis)) {
        r.add("DFM-MANIFOLD-CROSS", "error",
              "manifold_cross_drill_compound: drill1_axis and drill2_axis must be perpendicular");
    }

    if (in.plug_M.empty()) {
        r.add("DFM-MANIFOLD-PLUG", "error",
              "manifold_cross_drill_compound: plug_M is empty");
    } else {
        const PlugEntry* plug = lookupPlug(in.plug_M);
        if (!plug) {
            r.add("DFM-MANIFOLD-PLUG", "error",
                  "manifold_cross_drill_compound: unknown plug_M '" + in.plug_M +
                  "' (supported: M6/M8/M10/M12)");
        } else {
            if (plug->seat_dia_mm <= in.drill_dia_mm) {
                r.add("DFM-MANIFOLD-PLUG", "error",
                      "manifold_cross_drill_compound: plug seat_dia " +
                      std::to_string(plug->seat_dia_mm) +
                      " mm <= drill_dia " + std::to_string(in.drill_dia_mm) +
                      " mm (plug cannot seat against drill bore)");
            }
            if (in.drill_dia_mm < plug->min_drill_dia_mm) {
                r.add("DFM-MANIFOLD-PLUG", "warning",
                      "manifold_cross_drill_compound: drill_dia " +
                      std::to_string(in.drill_dia_mm) +
                      " mm below recommended min for plug " + in.plug_M);
            }
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
        std::string msg = "manifold_cross_drill_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const PlugEntry* plug = lookupPlug(in.plug_M);
    if (!plug) throw SkillError("manifold_cross_drill_compound: plug lookup unexpectedly failed");

    // Bounding diagonal: use this to make the through-holes safely overshoot.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));
    const double overshoot = bboxDiag + 2.0;

    const gp_Pnt cross(in.drill1_origin_x_mm,
                       in.drill1_origin_y_mm,
                       in.drill1_origin_z_mm);

    // Start each drill back along its negative axis so we go cleanly through.
    auto walkBack = [&](const gp_Pnt& p, const gp_Dir& d, double dist) {
        return gp_Pnt(p.X() - d.X() * dist,
                      p.Y() - d.Y() * dist,
                      p.Z() - d.Z() * dist);
    };

    const double drillR = in.drill_dia_mm / 2.0;
    const double seatR  = plug->seat_dia_mm / 2.0;

    // ── Sub-cut #1: drill1 through-hole ─────────────────────────────────
    const gp_Pnt s1 = walkBack(cross, in.drill1_axis, overshoot / 2.0);
    const gp_Ax2 ax1(s1, in.drill1_axis);
    const TopoDS_Shape drill1Tool = pr::cylinder(ax1, drillR, overshoot);

    // ── Sub-cut #2: drill2 through-hole ─────────────────────────────────
    const gp_Pnt s2 = walkBack(cross, in.drill2_axis, overshoot / 2.0);
    const gp_Ax2 ax2(s2, in.drill2_axis);
    const TopoDS_Shape drill2Tool = pr::cylinder(ax2, drillR, overshoot);

    // ── Sub-cut #3: plug counterbore on +drill1 end face ────────────────
    //
    // For axis-aligned drill axes (the common case), we read the +end face
    // plane directly from the bbox (xMax for +X axis, yMax for +Y, etc.).
    // For arbitrary axes we approximate face_dist as bbox_diag/2 from the
    // cross point — a conservative outside estimate (the counterbore tool
    // will still span across the actual face thanks to the overhang).
    const double seatOverhang = 0.05;

    auto endFaceDist = [&](const gp_Dir& d) {
        // Distance from cross point to the +end face along d.
        // For unit-axis d this is simply (max - cross) along that axis.
        if (std::abs(std::abs(d.X()) - 1.0) < 1e-6)
            return (d.X() > 0 ? xMax - cross.X() : cross.X() - xMin);
        if (std::abs(std::abs(d.Y()) - 1.0) < 1e-6)
            return (d.Y() > 0 ? yMax - cross.Y() : cross.Y() - yMin);
        if (std::abs(std::abs(d.Z()) - 1.0) < 1e-6)
            return (d.Z() > 0 ? zMax - cross.Z() : cross.Z() - zMin);
        // Arbitrary direction — fall back to bboxDiag / 2 (covers cube).
        return bboxDiag / 2.0;
    };

    auto seatToolFor = [&](const gp_Dir& d) {
        const double faceDist = endFaceDist(d);
        // Cone start (axis Location) sits INSIDE the block at face -
        // plug_seat_depth.  Direction = d (cylinder grows toward face +
        // overhang).  Total height = plug_seat_depth + seatOverhang so
        // the cone protrudes seatOverhang outside the face — guarantees
        // a clean Boolean cut at the face boundary.
        const gp_Pnt start(
            cross.X() + d.X() * (faceDist - in.plug_seat_depth_mm),
            cross.Y() + d.Y() * (faceDist - in.plug_seat_depth_mm),
            cross.Z() + d.Z() * (faceDist - in.plug_seat_depth_mm));
        const gp_Ax2 ax(start, d);
        return pr::cylinder(ax, seatR, in.plug_seat_depth_mm + seatOverhang);
    };

    const TopoDS_Shape cb1Tool = seatToolFor(in.drill1_axis);
    const TopoDS_Shape cb2Tool = seatToolFor(in.drill2_axis);

    // Pre-fuse concentric overlapping cutter pairs (drill1+cb1, drill2+cb2)
    // before the big cut to avoid OCCT cut-on-compound interior misses.
    const TopoDS_Shape leg1 = pr::fuse(drill1Tool, cb1Tool);
    const TopoDS_Shape leg2 = pr::fuse(drill2Tool, cb2Tool);
    const TopoDS_Shape unified = pr::fuse(leg1, leg2);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "drill1_axis", { in.drill1_axis.X(), in.drill1_axis.Y(), in.drill1_axis.Z() } },
        { "drill2_axis", { in.drill2_axis.X(), in.drill2_axis.Y(), in.drill2_axis.Z() } },
        { "drill1_origin_x_mm", in.drill1_origin_x_mm },
        { "drill1_origin_y_mm", in.drill1_origin_y_mm },
        { "drill1_origin_z_mm", in.drill1_origin_z_mm },
        { "drill_dia_mm",       in.drill_dia_mm },
        { "plug_M",             in.plug_M },
        { "plug_seat_depth_mm", in.plug_seat_depth_mm },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "subfeature_count",   4 },           // 2 drills + 2 counterbores
        { "drill_dia_mm",       in.drill_dia_mm },
        { "plug_M",             in.plug_M },
        { "plug_seat_dia_mm",   plug->seat_dia_mm },     // derived
        { "plug_seat_depth_mm", in.plug_seat_depth_mm },
        { "cross_point",        { in.drill1_origin_x_mm,
                                  in.drill1_origin_y_mm,
                                  in.drill1_origin_z_mm } },
        { "drill1_axis", { in.drill1_axis.X(), in.drill1_axis.Y(), in.drill1_axis.Z() } },
        { "drill2_axis", { in.drill2_axis.X(), in.drill2_axis.Y(), in.drill2_axis.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "drill;drill;counterbore;counterbore";
    tooling.tool_dia_mm = in.drill_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    // Approximate material removed: two through-cylinders of length ≈ bbox
    // diagonal, minus their intersection, plus two seat counterbores.
    const double drillLen = bboxDiag;  // pessimistic
    const double drillVol = M_PI * drillR * drillR * drillLen * 2.0
                          - M_PI * drillR * drillR * (2.0 * drillR);  // shared cross
    const double seatVol  = M_PI * seatR * seatR * in.plug_seat_depth_mm * 2.0
                          - M_PI * drillR * drillR * in.plug_seat_depth_mm * 2.0;
    tooling.stock_removed_mm3 = std::max(0.0, drillVol) + std::max(0.0, seatVol);
    tooling.est_cycle_time_s = std::max(2.0, drillLen / 25.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" }, { "tool_dia_mm", in.drill_dia_mm },
              { "axis", { in.drill1_axis.X(), in.drill1_axis.Y(), in.drill1_axis.Z() } } },
            { { "tool_type", "drill" }, { "tool_dia_mm", in.drill_dia_mm },
              { "axis", { in.drill2_axis.X(), in.drill2_axis.Y(), in.drill2_axis.Z() } } },
            { { "tool_type", "counterbore" }, { "tool_dia_mm", plug->seat_dia_mm },
              { "depth_mm", in.plug_seat_depth_mm },
              { "for", in.plug_M } },
            { { "tool_type", "counterbore" }, { "tool_dia_mm", plug->seat_dia_mm },
              { "depth_mm", in.plug_seat_depth_mm },
              { "for", in.plug_M } },
        } },
        { "note", "two perpendicular through-drills + two plug seats" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::manifold_cross_drill_compound applied: dia={} plug={} seat_d={}",
                  in.drill_dia_mm, in.plug_M, in.plug_seat_depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::manifold_cross_drill_compound
