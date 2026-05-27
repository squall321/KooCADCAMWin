// @lat: [[engine/skills#engrave_text]]
//
// ─── APPROXIMATION NOTE (slice 2) ────────────────────────────────────────
//
// Real text engraving traces glyph outlines from a font (TrueType/OpenType)
// and cuts along those wires.  Doing it in B-rep requires:
//
//   MISSING PRIMITIVE:
//       prim::glyphProfile(
//           char                ch,
//           double              font_size_mm,
//           const gp_Ax2&       baseline_frame);    // -> TopoDS_Wire (or face)
//
//       prim::engraveWire(
//           const TopoDS_Wire&  glyph_path,
//           const gp_Dir&       extrude_dir,
//           double              depth_mm);          // -> TopoDS_Shape cutter
//
//   The full pipeline is:
//
//     1. Rasterize a glyph (e.g. via FreeType → polyline → BRepBuilderAPI_
//        MakeWire), oriented in entry_face's plane.
//     2. Extrude that wire by depth_mm along the inward face normal using
//        BRepPrimAPI_MakePrism.
//     3. Cut the resulting prism out of the workpiece (one cut per glyph,
//        bundled into prim::cutMany).
//
//   None of those font primitives are available yet (slice 4 work).  Until
//   they land, engrave_text::apply() models the engraving as N small
//   cylindrical "spot" cuts (one per character) spaced by font_size_mm
//   along +X.  The spot diameter equals stroke_width_mm and the depth
//   equals depth_mm — geometrically minimal but it produces a valid solid
//   that records the engrave intent.
//
//   Swap the spot-cut loop for the real font wire + extrusion pipeline
//   once prim::glyphProfile + prim::engraveWire ship.
//
// ─────────────────────────────────────────────────────────────────────────

#include "engrave_text.hpp"

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

namespace koocadcam::skill::engrave_text {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.stroke_width_mm < 0.15) {
        r.add("DFM-019", "error",
              "engrave_text stroke_width_mm " + std::to_string(in.stroke_width_mm) +
              " < 0.15 mm (min V-cutter)");
    }
    if (in.depth_mm < 0.05 || in.depth_mm > 0.5) {
        r.add("DFM-ENGRAVE-DEPTH", "error",
              "engrave_text depth_mm " + std::to_string(in.depth_mm) +
              " not in [0.05, 0.5]");
    }
    if (in.text.empty()) {
        r.add("DFM-INPUT", "error", "engrave_text text must not be empty");
    }
    if (in.font_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "engrave_text font_size_mm must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "engrave_text DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("engrave_text: entry_face datum unresolved");

    // Approximate the engrave with N small cylindrical spot cuts.  For each
    // character index i, drill at:
    //   (position_x + i * font_size * dir.X,
    //    position_y + i * font_size * dir.Y,
    //    top of workpiece)
    //
    // depth = depth_mm, diameter = stroke_width_mm.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.02;
    const double radius    = in.stroke_width_mm / 2.0;
    const double cutterH   = in.depth_mm + kOverhang;

    std::vector<TopoDS_Shape> spots;
    spots.reserve(in.text.size());
    for (size_t i = 0; i < in.text.size(); ++i) {
        const double cx = in.position_x_mm +
                          static_cast<double>(i) * in.font_size_mm * in.direction.X();
        const double cy = in.position_y_mm +
                          static_cast<double>(i) * in.font_size_mm * in.direction.Y();
        const gp_Pnt top(cx, cy, zMax + kOverhang);
        const gp_Ax2 ax(top, gp_Dir(0.0, 0.0, -1.0));
        spots.push_back(pr::cylinder(ax, radius, cutterH));
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), spots);

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "direction",       { in.direction.X(), in.direction.Y(), in.direction.Z() } },
        { "text",            in.text },
        { "font_size_mm",    in.font_size_mm },
        { "stroke_width_mm", in.stroke_width_mm },
        { "depth_mm",        in.depth_mm },
    };
    json pattern = {
        { "kind",            kSkillId },
        { "char_count",      in.text.size() },
        { "font_size_mm",    in.font_size_mm },
        { "stroke_width_mm", in.stroke_width_mm },
        { "depth_mm",        in.depth_mm },
        { "geometry",        "spot_approximation" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "v_cutter";
    tooling.tool_dia_mm       = in.stroke_width_mm;
    tooling.tool_length_mm    = std::max(10.0, in.depth_mm * 5.0 + 5.0);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 800.0;
    tooling.feed_per_tooth_mm = 0.01;
    tooling.stock_removed_mm3 = M_PI * radius * radius * in.depth_mm *
                                static_cast<double>(in.text.size());
    tooling.est_cycle_time_s  = std::max(1.0,
                                static_cast<double>(in.text.size()) * 0.5);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::engrave_text applied: '{}' chars={} stroke={} depth={}",
                  in.text, in.text.size(), in.stroke_width_mm, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Without font-aware analysis, recognize() can only:
//   (a) Replay metadata signatures (full confidence).
//   (b) Heuristically detect rows of small shallow cylindrical features
//       (low confidence).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // (a) Metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // (b) Heuristic: collect small shallow cylinders.  If ≥ 2 with similar
    // radius collinear along an axis, propose a candidate.
    struct Spot {
        int    cylIdx;
        double radius;
        double depth;
        gp_Pnt center;
    };
    std::vector<Spot> spots;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const double r = cyl.Radius();
        if (r * 2.0 >= 1.0) continue;  // bigger than 1 mm dia → not engrave
        // depth ≈ cylinder face V parameter span (height)
        const double vSpan = surf.LastVParameter() - surf.FirstVParameter();
        if (vSpan <= 0.0 || vSpan > 0.5) continue;  // shallow only
        spots.push_back({ fIdx, r, vSpan, cyl.Axis().Location() });
    }
    if (spots.size() < 2) return out;

    // Look for a roughly collinear, equally spaced cluster.  Cheap heuristic:
    // sort by X, accept if all neighbours have ΔX > 0.5 mm and similar radii.
    std::sort(spots.begin(), spots.end(),
              [](const Spot& a, const Spot& b) { return a.center.X() < b.center.X(); });
    bool collinear = true;
    const double r0 = spots.front().radius;
    for (size_t i = 1; i < spots.size(); ++i) {
        if (std::abs(spots[i].radius - r0) > 0.05) { collinear = false; break; }
        if (std::abs(spots[i].center.Y() - spots.front().center.Y()) > 0.5) {
            collinear = false;
            break;
        }
    }
    if (!collinear) return out;

    const double font_size_guess = (spots.size() >= 2)
        ? (spots.back().center.X() - spots.front().center.X()) /
            static_cast<double>(spots.size() - 1)
        : 2.0;

    json recovered = {
        { "position_x_mm",   spots.front().center.X() },
        { "position_y_mm",   spots.front().center.Y() },
        { "direction",       { 1.0, 0.0, 0.0 } },
        { "text",            std::string(spots.size(), '?') },
        { "font_size_mm",    font_size_guess },
        { "stroke_width_mm", 2.0 * r0 },
        { "depth_mm",        spots.front().depth },
    };
    json matched = {
        { "spot_count", spots.size() },
    };
    out.push_back(RecognizedFeature{
        kSkillId, recovered, /*confidence*/ 0.30, matched
    });
    return out;
}

}  // namespace koocadcam::skill::engrave_text
