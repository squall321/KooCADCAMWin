// @lat: [[engine/skills#terminal_block_post]]

#include "terminal_block_post.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::terminal_block_post {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.stud_dia_mm < 2.0) {
        r.add("DFM-IEC60947", "error",
              "stud_dia " + std::to_string(in.stud_dia_mm) +
              " mm < M2 minimum (2.0 mm)");
    }
    if (in.stud_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "stud_height_mm must be > 0");
    }
    if (in.shoulder_dia_mm <= in.stud_dia_mm) {
        r.add("DFM-POST-GEOM", "error",
              "shoulder_dia must be > stud_dia");
    }
    if (in.skirt_dia_mm <= in.shoulder_dia_mm) {
        r.add("DFM-POST-GEOM", "error",
              "skirt_dia must be > shoulder_dia");
    }
    if (in.shoulder_height_mm <= 0.0 || in.skirt_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "shoulder_height and skirt_height must be > 0");
    }
    if (in.skirt_height_mm < 5.0) {
        r.add("DFM-IEC60664", "error",
              "skirt_height " + std::to_string(in.skirt_height_mm) +
              " mm < 5 mm creepage distance (IEC 60664 50V CAT II)");
    }
    if (in.wire_hole_dia_mm < 1.0) {
        r.add("DFM-002", "error",
              "wire_hole_dia " + std::to_string(in.wire_hole_dia_mm) +
              " mm < 1.0 mm");
    }
    if (in.wire_hole_dia_mm >= in.skirt_dia_mm - in.shoulder_dia_mm) {
        // Wire hole would either intersect shoulder or break the skirt.
        // It must fit cleanly through the skirt wall — i.e. its diameter
        // must be less than (skirt_dia - shoulder_dia).
        r.add("DFM-POST-WIRE", "error",
              "wire_hole_dia must be < (skirt_dia - shoulder_dia)");
    }
    if (in.wire_hole_z_mm <= 0.0 ||
        in.wire_hole_z_mm + in.wire_hole_dia_mm / 2.0 >= in.skirt_height_mm) {
        r.add("DFM-POST-WIRE", "error",
              "wire_hole_z out of skirt height bounds");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Volume math (delta = OUTPUT - INPUT, since post is ADDITIVE):
//   stud      : π·(stud_dia/2)²·stud_height
//   shoulder  : π·(shoulder_dia/2)²·shoulder_height
//   skirt     : π·((skirt_dia/2)² - (shoulder_dia/2)²)·skirt_height
//   wires (-) : 2 × π·(wire_dia/2)²·(skirt_dia/2 - shoulder_dia/2) ... subtracted

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "terminal_block_post DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("terminal_block_post: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() <= 0) {
        throw SkillError("terminal_block_post: only axis_dir = +Z supported (additive)");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zBase = zMax;  // sit on top face

    // 1) Skirt (largest, sits on the base).  Build first so unions are stable.
    const gp_Pnt skirtBase(in.position_x_mm, in.position_y_mm, zBase);
    const gp_Ax2 skirtAx(skirtBase, gp_Dir(0, 0, 1));
    const TopoDS_Shape skirt = pr::cylinder(
        skirtAx, in.skirt_dia_mm / 2.0, in.skirt_height_mm);

    // 2) Shoulder (above the skirt — but coaxial; we model the post as
    //    skirt up to skirt_height, shoulder sitting on the skirt top
    //    + stud sitting on the shoulder).  But for "shoulder is the part
    //    that retains the post inside the insulator", the canonical layout
    //    is: shoulder is the COLLAR at the BASE of the stud, ABOVE the
    //    skirt.  We place shoulder at z=zBase+skirt_height extending up
    //    by shoulder_height, and stud on top of that.
    const gp_Pnt shoulderBase(
        in.position_x_mm, in.position_y_mm, zBase + in.skirt_height_mm);
    const gp_Ax2 shoulderAx(shoulderBase, gp_Dir(0, 0, 1));
    const TopoDS_Shape shoulder = pr::cylinder(
        shoulderAx, in.shoulder_dia_mm / 2.0, in.shoulder_height_mm);

    // 3) Stud (top, narrowest).
    const gp_Pnt studBase(
        in.position_x_mm, in.position_y_mm,
        zBase + in.skirt_height_mm + in.shoulder_height_mm);
    const gp_Ax2 studAx(studBase, gp_Dir(0, 0, 1));
    const TopoDS_Shape stud = pr::cylinder(
        studAx, in.stud_dia_mm / 2.0, in.stud_height_mm);

    // Fuse additive parts onto the base workpiece.
    TopoDS_Shape composite = pr::fuse(wp.shape(), skirt);
    composite = pr::fuse(composite, shoulder);
    composite = pr::fuse(composite, stud);

    // 4) Two wire holes through the skirt — perpendicular to stud axis (X
    //    direction), diametrically opposed.  Each hole spans the full skirt
    //    diameter at z = wire_hole_z (above the base).
    const double zHole = zBase + in.wire_hole_z_mm;
    const double rWire = in.wire_hole_dia_mm / 2.0;
    const double holeLen = in.skirt_dia_mm + 2.0;  // pierce both walls

    // Hole 1 — along +X (entry on +X, exit on -X) — single cylinder along X.
    {
        const gp_Pnt origin1(
            in.position_x_mm - holeLen / 2.0,
            in.position_y_mm,
            zHole);
        const gp_Ax2 holeAx(origin1, gp_Dir(1, 0, 0));
        const TopoDS_Shape hole1 = pr::cylinder(holeAx, rWire, holeLen);

        const gp_Pnt origin2(
            in.position_x_mm,
            in.position_y_mm - holeLen / 2.0,
            zHole);
        const gp_Ax2 holeAx2(origin2, gp_Dir(0, 1, 0));
        const TopoDS_Shape hole2 = pr::cylinder(holeAx2, rWire, holeLen);

        // Fuse both holes into a single cutter (perpendicular cross).
        const TopoDS_Shape holes = pr::fuse(hole1, hole2);
        composite = pr::cut(composite, holes);
    }

    json params = {
        { "entry_face_id",      *entryId },
        { "position_x_mm",      in.position_x_mm },
        { "position_y_mm",      in.position_y_mm },
        { "axis_dir",           { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "stud_dia_mm",        in.stud_dia_mm },
        { "stud_height_mm",     in.stud_height_mm },
        { "shoulder_dia_mm",    in.shoulder_dia_mm },
        { "shoulder_height_mm", in.shoulder_height_mm },
        { "skirt_dia_mm",       in.skirt_dia_mm },
        { "skirt_height_mm",    in.skirt_height_mm },
        { "wire_hole_dia_mm",   in.wire_hole_dia_mm },
        { "wire_hole_z_mm",     in.wire_hole_z_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },
        { "stud_dia_mm",         in.stud_dia_mm },
        { "shoulder_dia_mm",     in.shoulder_dia_mm },
        { "skirt_dia_mm",        in.skirt_dia_mm },
        { "wire_hole_count",     2 },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "turn;mill;drill";
    tooling.tool_dia_mm       = in.skirt_dia_mm;
    tooling.tool_length_mm    = in.stud_height_mm + in.shoulder_height_mm + in.skirt_height_mm + 5.0;
    tooling.tool_material     = "brass_billet";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double rStud = in.stud_dia_mm / 2.0;
    const double rSh   = in.shoulder_dia_mm / 2.0;
    const double rSk   = in.skirt_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        M_PI * rStud * rStud * in.stud_height_mm
      + M_PI * rSh   * rSh   * in.shoulder_height_mm
      + M_PI * rSk   * rSk   * in.skirt_height_mm
      // wire-hole removed material — each hole crosses the full skirt
      // diameter (2*rSk), there are 2 perpendicular holes.
      - 2.0 * M_PI * rWire * rWire * (2.0 * rSk);
    tooling.est_cycle_time_s = std::max(4.0, in.skirt_height_mm * 0.5 + 5.0);
    tooling.extra = {
        { "standard",       "UL 1059 / IEC 60947 / IEC 60664" },
        { "operation_note", "Turn skirt/shoulder/stud, drill 2 cross wire holes" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(composite, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::terminal_block_post applied: stud d{}xh{} faces {}->{}",
                  in.stud_dia_mm, in.stud_height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────


std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay (high confidence).
    for (const auto& sig : wp.features()) {
        if (sig.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = sig.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }

    // Geometric heuristic (PRIMARY): three COAXIAL vertical cylinders (stud,
    // shoulder, skirt) of strictly increasing radius STACKED in increasing
    // Z order (skirt at bottom, shoulder above, stud at top) PLUS two
    // perpendicular wire holes piercing the skirt at the same Z.
    struct Cyl { int faceIdx; double radius; gp_Ax1 axis; gp_Pnt center; };
    std::vector<Cyl> verticalCyls, horizontalCyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(d.Z()) > 0.95)
            verticalCyls.push_back({ i, c.Radius(), c.Axis(), wp.faceCenter(i) });
        else if (std::abs(d.Z()) < 0.05)
            horizontalCyls.push_back({ i, c.Radius(), c.Axis(), wp.faceCenter(i) });
    }
    if (verticalCyls.size() < 3) return out;

    // True coaxial check (parallel + same axis-line position).
    auto coaxialAt = [](const gp_Ax1& a, const gp_Ax1& b, double tolMm = 0.5) {
        if (!a.Direction().IsParallel(b.Direction(), 1e-2)) return false;
        const gp_Vec v(b.Location(), a.Location());
        const gp_Vec d(b.Direction());
        const gp_Vec proj = d * v.Dot(d);
        return (v - proj).Magnitude() < tolMm;
    };

    std::vector<bool> used(verticalCyls.size(), false);
    for (size_t i = 0; i < verticalCyls.size(); ++i) {
        if (used[i]) continue;
        std::vector<size_t> group{ i };
        used[i] = true;
        for (size_t j = i + 1; j < verticalCyls.size(); ++j) {
            if (used[j]) continue;
            if (coaxialAt(verticalCyls[i].axis, verticalCyls[j].axis)) {
                group.push_back(j);
                used[j] = true;
            }
        }
        if (group.size() < 3) continue;

        // Sort group by radius for diameter selection,
        // and by Z for height-order verification.
        std::vector<size_t> sortedByR = group;
        std::sort(sortedByR.begin(), sortedByR.end(),
                  [&](size_t a, size_t b){
                      return verticalCyls[a].radius < verticalCyls[b].radius;
                  });
        std::vector<size_t> sortedByZ = group;
        std::sort(sortedByZ.begin(), sortedByZ.end(),
                  [&](size_t a, size_t b){
                      return verticalCyls[a].center.Z() < verticalCyls[b].center.Z();
                  });

        // Pick rStud (smallest), rSk (largest), rSh (middle distinct).
        const double rStud = verticalCyls[sortedByR.front()].radius;
        const double rSk   = verticalCyls[sortedByR.back()].radius;
        if (rSk - rStud < 0.2) continue;       // need distinct radii
        double rSh = rStud;
        for (size_t idx : sortedByR) {
            const double r = verticalCyls[idx].radius;
            if (r > rStud + 0.1 && r < rSk - 0.1) { rSh = r; break; }
        }
        if (rSh <= rStud + 0.05 || rSh >= rSk - 0.05) continue;

        // Verify height ordering: skirt (largest R) should be at LOW Z,
        // stud (smallest R) at HIGH Z.  Use sortedByZ[0] vs sortedByZ.back().
        const double zLow  = verticalCyls[sortedByZ.front()].center.Z();
        const double zHigh = verticalCyls[sortedByZ.back()].center.Z();
        const bool ordered = (zHigh - zLow) > 1e-3;

        // Count perpendicular cylinders crossing near this group's axis,
        // AND whose Z is within the skirt height (low/middle of stack).
        int wireHits = 0;
        const gp_Pnt axisLoc = verticalCyls[i].axis.Location();
        for (const auto& hc : horizontalCyls) {
            const gp_Pnt p = hc.center;
            const double dx = p.X() - axisLoc.X();
            const double dy = p.Y() - axisLoc.Y();
            if (std::hypot(dx, dy) < rSk * 1.2 &&
                p.Z() < zLow + (zHigh - zLow) * 0.6) ++wireHits;
        }

        int subScore = 0;
        if (group.size() >= 3) ++subScore;
        if (ordered)           ++subScore;
        if (wireHits >= 1)     ++subScore;
        if (wireHits >= 2)     ++subScore;
        const double confidence = std::min(0.90, 0.55 + 0.09 * subScore);

        json recovered = {
            { "position_x_mm",      axisLoc.X() },
            { "position_y_mm",      axisLoc.Y() },
            { "axis_dir",           { 0.0, 0.0, 1.0 } },
            { "stud_dia_mm",        2.0 * rStud },
            { "shoulder_dia_mm",    2.0 * rSh },
            { "skirt_dia_mm",       2.0 * rSk },
            { "wire_hole_count",    wireHits },
        };
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = recovered;
        r.confidence       = confidence;
        r.matched_geometry = {
            { "vertical_cyl_count",  static_cast<int>(group.size()) },
            { "stack_ordered",       ordered },
            { "wire_hole_hits",      wireHits },
            { "subfeature_score",    subScore },
            { "source",              "geometric" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::terminal_block_post
