// @lat: [[engine/skills#governor_arm_with_pivot]]

#include "governor_arm_with_pivot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::governor_arm_with_pivot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.arm_length_mm < 10.0) {
        r.add("DFM-GOVARM-LENGTH", "error",
              "governor_arm_with_pivot: arm_length " +
              std::to_string(in.arm_length_mm) + " < 10 mm");
    }
    if (in.arm_width_mm < 4.0) {
        r.add("DFM-GOVARM-WIDTH", "error",
              "governor_arm_with_pivot: arm_width " +
              std::to_string(in.arm_width_mm) + " < 4 mm");
    }
    if (in.arm_thickness_mm < 3.0) {
        r.add("DFM-GOVARM-THICK", "error",
              "governor_arm_with_pivot: arm_thickness " +
              std::to_string(in.arm_thickness_mm) + " < 3 mm");
    }
    if (in.pivot_bore_dia_mm < 2.0) {
        r.add("DFM-002", "error",
              "governor_arm_with_pivot: pivot_bore_dia " +
              std::to_string(in.pivot_bore_dia_mm) + " < 2 mm");
    }
    if (in.pivot_bore_dia_mm > 0.7 * in.arm_width_mm) {
        r.add("DFM-GOVARM-WALL", "error",
              "governor_arm_with_pivot: pivot_bore_dia (" +
              std::to_string(in.pivot_bore_dia_mm) +
              ") > 0.7 × arm_width (" + std::to_string(in.arm_width_mm) +
              ") — wall too thin (ASME B5.16)");
    }
    if (in.ball_seat_dia_mm < 2.0) {
        r.add("DFM-002", "error",
              "governor_arm_with_pivot: ball_seat_dia " +
              std::to_string(in.ball_seat_dia_mm) + " < 2 mm");
    }
    if (in.ball_seat_dia_mm > 0.8 * in.arm_width_mm) {
        r.add("DFM-GOVARM-BALL", "error",
              "governor_arm_with_pivot: ball_seat_dia (" +
              std::to_string(in.ball_seat_dia_mm) +
              ") > 0.8 × arm_width — would break out");
    }
    if (in.ball_seat_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "governor_arm_with_pivot: ball_seat_depth must be > 0");
    }
    if (in.ball_seat_depth_mm > 0.7 * in.arm_thickness_mm) {
        r.add("DFM-GOVARM-DEPTH", "error",
              "governor_arm_with_pivot: ball_seat_depth (" +
              std::to_string(in.ball_seat_depth_mm) +
              ") > 0.7 × arm_thickness — would be a through-hole");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "governor_arm_with_pivot DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double overhang = 0.05;
    const double bottomZ  = in.corner_z_mm;
    const double topZ     = bottomZ + in.arm_thickness_mm;
    const double w        = in.arm_width_mm;

    // ── step 0: build L-arm geometry by fusing two perpendicular boxes ──
    //
    // Leg A (along +X): box from (corner_x - w/2, corner_y - w/2, bottomZ)
    //   with dx = arm_length + w/2, dy = w, dz = arm_thickness.
    // Leg B (along +Y): box from (corner_x - w/2, corner_y - w/2, bottomZ)
    //   with dx = w, dy = arm_length + w/2, dz = arm_thickness.
    // Their overlap is the corner square (w × w × arm_thickness).
    const gp_Pnt legAOrigin(in.corner_x_mm - w / 2.0,
                            in.corner_y_mm - w / 2.0,
                            bottomZ);
    const gp_Ax2 legAAx(legAOrigin, gp::DZ());
    const TopoDS_Shape legA = pr::box(legAAx,
        in.arm_length_mm + w / 2.0, w, in.arm_thickness_mm);

    const gp_Pnt legBOrigin(in.corner_x_mm - w / 2.0,
                            in.corner_y_mm - w / 2.0,
                            bottomZ);
    const gp_Ax2 legBAx(legBOrigin, gp::DZ());
    const TopoDS_Shape legB = pr::box(legBAx,
        w, in.arm_length_mm + w / 2.0, in.arm_thickness_mm);

    const TopoDS_Shape armLshape = pr::fuse(legA, legB);
    const TopoDS_Shape fused     = pr::fuse(wp.shape(), armLshape);

    // ── step 1: CUT central pivot bore at corner ────────────────────────
    const gp_Pnt pvtOrigin(in.corner_x_mm, in.corner_y_mm, bottomZ - overhang);
    const gp_Ax2 pvtAx(pvtOrigin, gp::DZ());
    const TopoDS_Shape pivotTool = pr::cylinder(
        pvtAx, in.pivot_bore_dia_mm / 2.0, in.arm_thickness_mm + 2.0 * overhang);

    // ── step 2 & 3: ball-seat pockets at each arm end ───────────────────
    //
    // End A is at (corner_x + arm_length, corner_y).  End B is at
    // (corner_x, corner_y + arm_length).  Each is a partial-depth cylinder
    // entering from topZ going -Z by ball_seat_depth.
    const gp_Pnt bsAOrigin(in.corner_x_mm + in.arm_length_mm,
                           in.corner_y_mm,
                           topZ - in.ball_seat_depth_mm);
    const gp_Ax2 bsAAx(bsAOrigin, gp::DZ());
    const TopoDS_Shape bsATool = pr::cylinder(
        bsAAx, in.ball_seat_dia_mm / 2.0,
        in.ball_seat_depth_mm + overhang);

    const gp_Pnt bsBOrigin(in.corner_x_mm,
                           in.corner_y_mm + in.arm_length_mm,
                           topZ - in.ball_seat_depth_mm);
    const gp_Ax2 bsBAx(bsBOrigin, gp::DZ());
    const TopoDS_Shape bsBTool = pr::cylinder(
        bsBAx, in.ball_seat_dia_mm / 2.0,
        in.ball_seat_depth_mm + overhang);

    TopoDS_Shape combined = pr::fuse(pivotTool, bsATool);
    combined = pr::fuse(combined, bsBTool);
    const TopoDS_Shape newShape = pr::cut(fused, combined);

    // ── signature ────────────────────────────────────────────────────────
    json params = {
        { "arm_length_mm",      in.arm_length_mm },
        { "arm_width_mm",       in.arm_width_mm },
        { "arm_thickness_mm",   in.arm_thickness_mm },
        { "pivot_bore_dia_mm",  in.pivot_bore_dia_mm },
        { "ball_seat_dia_mm",   in.ball_seat_dia_mm },
        { "ball_seat_depth_mm", in.ball_seat_depth_mm },
        { "corner_x_mm",        in.corner_x_mm },
        { "corner_y_mm",        in.corner_y_mm },
        { "corner_z_mm",        in.corner_z_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        // 2 L-legs (fused) + pivot + 2 ball seats = 5 subfeatures
        { "subfeature_count",    5 },
        { "axial_cyl_face_count", 3 },     // pivot + 2 ball seats
        { "pivot_bore_dia_mm",   in.pivot_bore_dia_mm },
        { "ball_seat_dia_mm",    in.ball_seat_dia_mm },
        { "arm_length_mm",       in.arm_length_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill;ball_mill";
    tooling.tool_dia_mm       = std::max(in.pivot_bore_dia_mm, in.ball_seat_dia_mm);
    tooling.tool_length_mm    = in.arm_thickness_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double rP = in.pivot_bore_dia_mm / 2.0;
    const double rB = in.ball_seat_dia_mm  / 2.0;
    const double vPivot = M_PI * rP * rP * in.arm_thickness_mm;
    const double vBall  = M_PI * rB * rB * in.ball_seat_depth_mm;
    tooling.stock_removed_mm3 = vPivot + 2.0 * vBall;
    tooling.est_cycle_time_s  = std::max(3.0,
        in.arm_thickness_mm * 0.2 + in.ball_seat_depth_mm * 0.3 + 2.0);
    tooling.extra = {
        { "added_material_mm3",
          w * (2.0 * in.arm_length_mm + w / 2.0 + w / 2.0) * in.arm_thickness_mm
          - w * w * in.arm_thickness_mm },  // L-area times thickness, minus corner overlap
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.pivot_bore_dia_mm },
              { "depth_mm",    in.arm_thickness_mm },
              { "note",        "pivot through-bore" } },
            { { "tool_type", "ball_mill" },
              { "tool_dia_mm", in.ball_seat_dia_mm },
              { "depth_mm",    in.ball_seat_depth_mm },
              { "pass_count",  2 },
              { "note",        "ball-seat pockets at L-arm ends (ASME B18.3)" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::governor_arm_with_pivot applied: L={} w={} t={} faces {}→{}",
                  in.arm_length_mm, in.arm_width_mm, in.arm_thickness_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // metadata fallback
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "feature_history" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // geometric: ≥ 3 axial cylinders; the deepest = pivot bore; the other
    // 2 (shallower, off-centre) = ball seats.
    struct CylInfo { int faceIdx; double radius; gp_Pnt loc; gp_Dir dir; };
    std::vector<CylInfo> axCyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        if (std::abs(std::abs(cy.Axis().Direction().Z()) - 1.0) > 1e-2) continue;
        axCyls.push_back({ i, cy.Radius(), cy.Axis().Location(),
                          cy.Axis().Direction() });
    }
    if (axCyls.size() < 3) return out;

    // Find one cyl whose XY position is "corner" — i.e. has TWO others at
    // roughly the same distance away in the +X and +Y direction (the
    // ball-seat ends).
    int bestPivot = -1, bestBsA = -1, bestBsB = -1;
    double bestScore = 1e30;
    for (size_t p = 0; p < axCyls.size(); ++p) {
        for (size_t a = 0; a < axCyls.size(); ++a) {
            if (a == p) continue;
            for (size_t b = 0; b < axCyls.size(); ++b) {
                if (b == p || b == a) continue;
                const double dxA = axCyls[a].loc.X() - axCyls[p].loc.X();
                const double dyA = axCyls[a].loc.Y() - axCyls[p].loc.Y();
                const double dxB = axCyls[b].loc.X() - axCyls[p].loc.X();
                const double dyB = axCyls[b].loc.Y() - axCyls[p].loc.Y();
                const double rA = std::sqrt(dxA*dxA + dyA*dyA);
                const double rB = std::sqrt(dxB*dxB + dyB*dyB);
                if (rA < 5.0 || rB < 5.0) continue;
                // L-arm: A is on +X axis-ish, B on +Y axis-ish.
                if (std::abs(dxA) < std::abs(dyA)) continue;   // A not X-leg
                if (std::abs(dyB) < std::abs(dxB)) continue;   // B not Y-leg
                // Score: arms should be roughly equal length, and the two
                // ball seats should be of similar radius.
                const double lenDiff = std::abs(rA - rB);
                const double radDiff = std::abs(axCyls[a].radius - axCyls[b].radius);
                const double score   = lenDiff + 5.0 * radDiff;
                if (score < bestScore) {
                    bestScore = score;
                    bestPivot = static_cast<int>(p);
                    bestBsA   = static_cast<int>(a);
                    bestBsB   = static_cast<int>(b);
                }
            }
        }
    }
    if (bestPivot < 0) return out;

    const double dxA = axCyls[bestBsA].loc.X() - axCyls[bestPivot].loc.X();
    const double dyB = axCyls[bestBsB].loc.Y() - axCyls[bestPivot].loc.Y();
    const double armLen = (std::abs(dxA) + std::abs(dyB)) / 2.0;

    json recovered = {
        { "arm_length_mm",      armLen },
        { "arm_width_mm",       2.0 * axCyls[bestPivot].radius * 1.5 }, // approx
        { "arm_thickness_mm",   0.0 },
        { "pivot_bore_dia_mm",  2.0 * axCyls[bestPivot].radius },
        { "ball_seat_dia_mm",
          axCyls[bestBsA].radius + axCyls[bestBsB].radius },           // 2 * mean(r)
        { "ball_seat_depth_mm", 0.0 },
        { "corner_x_mm",        axCyls[bestPivot].loc.X() },
        { "corner_y_mm",        axCyls[bestPivot].loc.Y() },
        { "corner_z_mm",        axCyls[bestPivot].loc.Z() },
    };
    json matched = {
        { "pivot_face_id",   axCyls[bestPivot].faceIdx },
        { "ball_seat_a_id",  axCyls[bestBsA].faceIdx },
        { "ball_seat_b_id",  axCyls[bestBsB].faceIdx },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.7, matched });
    return out;
}

}  // namespace koocadcam::skill::governor_arm_with_pivot
