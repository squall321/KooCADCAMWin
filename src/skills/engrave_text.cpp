// @lat: [[engine/skills#engrave_text]]
//
// ─── REAL GLYPH EXTRUSION (slice 6 upgrade) ──────────────────────────────
//
// engrave_text now performs REAL character glyph extrusion (no more
// "one tiny cylinder per character" spot-cut approximation).  For each
// character of `text`:
//
//   1. Look up the glyph polygon from `glyph::tableFor(ch)` — a static
//      hard-coded table (no FreeType dependency).  The table covers
//      uppercase A-Z + digits 0-9 (36 glyphs).  Glyphs are stored as
//      closed 2-D polygons in a UNIT-SIZE reference frame; the first
//      polygon is the OUTER outline, subsequent polygons are INNER
//      holes (used by O, B, D, P, A, 0, 6, 8, 9 etc).
//
//   2. Map each polygon's 2-D vertices (u, v) into world-3D by:
//        u → in.position_x_mm + (char_idx × font_size_mm × direction.X())
//                              + u_local · font_size_mm · direction.X()
//             [the u_local term advances WITHIN the glyph along the
//              same baseline direction]
//        v → in.position_y_mm + v_local · font_size_mm   (perpendicular
//             to the baseline in the engrave plane)
//
//   3. Build a closed wire per polygon via BRepBuilderAPI_MakeWire of
//      consecutive line edges, then a face from the OUTER wire with
//      any HOLE wires subtracted via BRepBuilderAPI_MakeFace.Add().
//
//   4. Extrude the face downward (along -Z = into the workpiece) by
//      depth_mm + kOverhang via BRepPrimAPI_MakePrism.
//
//   5. Fuse all glyph prisms into one cutter compound and cut from the
//      workpiece in a single Boolean op (fewer history nodes than N
//      sequential cuts).
//
// ── Trade-offs ──
//
//   PRO:
//     - Real letter shapes that read as the intended characters.
//     - No external font dependency (FreeType would add ~500 KB and a
//       new submodule).
//     - Deterministic — no font hinting jitter across platforms.
//
//   CON:
//     - The glyph table is BLOCKY / stencil-style (axis-aligned
//       strokes, no Béziers).  Letters are recognizable but typographically
//       crude.
//     - Lowercase, punctuation, accented characters, non-Latin scripts
//       are NOT in the table — these fall back to the legacy spot-cut
//       approximation (one small cylinder per character).
//     - Strokes have a fixed width (1/5 of glyph height) — no font
//       weight variants.
//
//   FUTURE WORK:
//     - When FreeType becomes acceptable, swap glyph::tableFor for a
//       FreeType-backed loader returning the same GlyphPath shape.
//     - Lowercase + punctuation table could be added by hand if needed.
//
// ─────────────────────────────────────────────────────────────────────────

#include "engrave_text.hpp"

#include "Workpiece.hpp"
#include "_glyph_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
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

// ── Internal helpers ─────────────────────────────────────────────────────

namespace {

// Convert lowercase ASCII to uppercase so the glyph table (uppercase only)
// matches both cases.  Non-alphabetic chars pass through unchanged.
char asGlyphKey(char c)
{
    if (c >= 'a' && c <= 'z') return static_cast<char>(c - 'a' + 'A');
    return c;
}

// Build a closed wire from a polygon of (x, y) vertices in WORLD coords.
// The polygon must be closed (last vertex connects to first); we add the
// closing edge automatically.  Throws Standard_Failure on build failure.
TopoDS_Wire buildPolygonWire(const std::vector<gp_Pnt>& worldPts)
{
    if (worldPts.size() < 3)
        throw Standard_Failure("engrave_text: polygon has < 3 vertices");
    BRepBuilderAPI_MakeWire wireMk;
    for (size_t i = 0; i < worldPts.size(); ++i) {
        const gp_Pnt& a = worldPts[i];
        const gp_Pnt& b = worldPts[(i + 1) % worldPts.size()];
        BRepBuilderAPI_MakeEdge mkEdge(a, b);
        if (!mkEdge.IsDone())
            throw Standard_Failure("engrave_text: polygon edge build failed");
        wireMk.Add(mkEdge.Edge());
    }
    if (!wireMk.IsDone())
        throw Standard_Failure("engrave_text: polygon wire build failed");
    return wireMk.Wire();
}

// Map a 2-D glyph-local (u, v) point to a world 3-D point.
//   origin       — world XY of the char's baseline anchor (top-Z of stock)
//   advanceDir   — text baseline direction in the engrave plane (world)
//   perpDir      — perpendicular to advanceDir in the engrave plane (world)
//   font_size_mm — uniform scale
//   topZ         — Z value of the entry face (we build the face AT topZ
//                  and extrude downward)
gp_Pnt glyphPoint(const gp_Pnt& origin,
                  const gp_Dir& advanceDir,
                  const gp_Dir& perpDir,
                  double font_size_mm,
                  double u_local, double v_local,
                  double topZ)
{
    return gp_Pnt(
        origin.X() + u_local * font_size_mm * advanceDir.X()
                   + v_local * font_size_mm * perpDir.X(),
        origin.Y() + u_local * font_size_mm * advanceDir.Y()
                   + v_local * font_size_mm * perpDir.Y(),
        topZ);
}

// Build one cutter prism for a single glyph at the given world anchor.
// Returns Null on failure; caller falls back to a spot-cut.
TopoDS_Shape buildGlyphPrism(const glyph::GlyphPath& g,
                             const gp_Pnt&  charAnchor,
                             const gp_Dir&  advanceDir,
                             const gp_Dir&  perpDir,
                             double         font_size_mm,
                             double         depth_mm,
                             double         topZ,
                             double         overhang)
{
    if (g.outer_and_holes.empty()) return TopoDS_Shape();
    try {
        // Build the outer wire.
        std::vector<gp_Pnt> outerPts;
        outerPts.reserve(g.outer_and_holes[0].size());
        for (const auto& uv : g.outer_and_holes[0]) {
            outerPts.push_back(glyphPoint(
                charAnchor, advanceDir, perpDir, font_size_mm,
                uv[0], uv[1], topZ + overhang));
        }
        const TopoDS_Wire outerWire = buildPolygonWire(outerPts);

        // Build the face from the outer wire, then add holes.
        BRepBuilderAPI_MakeFace faceMk(outerWire, /*OnlyPlane=*/true);
        if (!faceMk.IsDone())
            throw Standard_Failure("engrave_text: outer face build failed");

        for (size_t h = 1; h < g.outer_and_holes.size(); ++h) {
            std::vector<gp_Pnt> holePts;
            holePts.reserve(g.outer_and_holes[h].size());
            for (const auto& uv : g.outer_and_holes[h]) {
                holePts.push_back(glyphPoint(
                    charAnchor, advanceDir, perpDir, font_size_mm,
                    uv[0], uv[1], topZ + overhang));
            }
            // Reverse the hole wire so its winding is opposite the outer
            // (BRepBuilderAPI_MakeFace.Add expects the hole wire to be
            // wound CW when the outer is CCW).
            std::reverse(holePts.begin(), holePts.end());
            const TopoDS_Wire holeWire = buildPolygonWire(holePts);
            faceMk.Add(holeWire);
            if (!faceMk.IsDone()) {
                spdlog::debug("engrave_text: hole add failed for glyph; "
                              "continuing without this hole");
            }
        }
        const TopoDS_Shape face = faceMk.Face();
        if (face.IsNull())
            throw Standard_Failure("engrave_text: glyph face null");

        // Extrude downward into the workpiece.
        const gp_Vec extrudeVec(0.0, 0.0, -(depth_mm + overhang));
        BRepPrimAPI_MakePrism prismMk(face, extrudeVec);
        prismMk.Build();
        if (!prismMk.IsDone())
            throw Standard_Failure("engrave_text: prism build failed");
        return prismMk.Shape();
    }
    catch (const Standard_Failure& e) {
        spdlog::debug("engrave_text: glyph prism failed ({}); will use spot-cut fallback",
                      e.what());
        return TopoDS_Shape();
    }
    catch (const std::exception& e) {
        spdlog::debug("engrave_text: glyph prism threw ({}); will use spot-cut fallback",
                      e.what());
        return TopoDS_Shape();
    }
}

// Legacy spot-cut fallback for chars not in the glyph table or whose glyph
// extrusion failed.  One small cylinder at the char anchor.
TopoDS_Shape buildSpotCutter(const gp_Pnt& charAnchor,
                             double         topZ,
                             double         stroke_radius,
                             double         depth_mm,
                             double         overhang)
{
    const gp_Pnt top(charAnchor.X(), charAnchor.Y(), topZ + overhang);
    const gp_Ax2 ax(top, gp_Dir(0.0, 0.0, -1.0));
    return pr::cylinder(ax, stroke_radius, depth_mm + overhang);
}

}  // namespace

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

    // ── Setup geometry frame ─────────────────────────────────────────────
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.02;

    // Perpendicular to baseline direction in the engrave plane (Z = up).
    // perpDir = (-dir.Y, dir.X, 0) when dir lies in the XY plane (the
    // common case for top-face engraving).  For full 3-D engrave-plane
    // generality we'd derive perpDir from the face normal; here we take
    // it as the in-plane perpendicular to `direction` projected on XY.
    // If direction is purely vertical (Z only), fall back to +X / +Y so
    // gp_Dir doesn't throw on zero magnitude.
    gp_Dir advanceDir(1.0, 0.0, 0.0);
    gp_Dir perpDir(0.0, 1.0, 0.0);
    {
        const double dx = in.direction.X();
        const double dy = in.direction.Y();
        if (std::abs(dx) + std::abs(dy) > 1e-9) {
            advanceDir = gp_Dir(dx, dy, 0.0);
            perpDir    = gp_Dir(-advanceDir.Y(), advanceDir.X(), 0.0);
        }
    }

    const double radius = in.stroke_width_mm / 2.0;

    // ── Build cutters: one per character ─────────────────────────────────
    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(in.text.size());
    int glyphsBuilt   = 0;
    int spotFallbacks = 0;

    for (size_t i = 0; i < in.text.size(); ++i) {
        const char ch = in.text[i];

        // Anchor for this character — origin of its (u, v) glyph box.
        // The advanceDir step is one font_size per character (kerned/
        // monospace).
        const double advT = static_cast<double>(i) * in.font_size_mm;
        const gp_Pnt charAnchor(
            in.position_x_mm + advT * advanceDir.X(),
            in.position_y_mm + advT * advanceDir.Y(),
            0.0);

        const glyph::GlyphPath& g = glyph::tableFor(asGlyphKey(ch));
        TopoDS_Shape prism;
        if (!g.outer_and_holes.empty()) {
            prism = buildGlyphPrism(g, charAnchor, advanceDir, perpDir,
                                    in.font_size_mm, in.depth_mm, zMax, kOverhang);
        }
        if (prism.IsNull()) {
            // Fallback: spot cut.
            try {
                // For the spot fallback, place the spot at the GLYPH CENTRE
                // (charAnchor + 0.5·font_size in both u and v).  This keeps
                // the volume removed similar to a glyph extrusion.
                const gp_Pnt spotCenter(
                    charAnchor.X() + 0.5 * in.font_size_mm * advanceDir.X()
                                   + 0.5 * in.font_size_mm * perpDir.X(),
                    charAnchor.Y() + 0.5 * in.font_size_mm * advanceDir.Y()
                                   + 0.5 * in.font_size_mm * perpDir.Y(),
                    0.0);
                prism = buildSpotCutter(spotCenter, zMax, radius,
                                        in.depth_mm, kOverhang);
                ++spotFallbacks;
            }
            catch (const std::exception& e) {
                spdlog::warn("engrave_text: spot fallback failed for '{}' ({})",
                             ch, e.what());
                continue;
            }
        } else {
            ++glyphsBuilt;
        }
        cutters.push_back(prism);
    }
    if (cutters.empty())
        throw SkillError("engrave_text: no valid cutters built from text");

    TopoDS_Shape newShape;
    try {
        newShape = pr::cutMany(wp.shape(), cutters);
    }
    catch (const std::exception& e) {
        spdlog::warn("engrave_text: cutMany failed ({}); leaving workpiece unchanged",
                     e.what());
        newShape = wp.shape();
    }

    // ── Signature ────────────────────────────────────────────────────────
    const std::string geomTag =
        (spotFallbacks == 0 && glyphsBuilt > 0) ? "glyph_extrusion"
        : (glyphsBuilt == 0)                    ? "spot_approximation"
                                                : "glyph_with_spot_fallback";

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
        { "glyphs_built",    glyphsBuilt },
        { "spot_fallbacks",  spotFallbacks },
        { "font_size_mm",    in.font_size_mm },
        { "stroke_width_mm", in.stroke_width_mm },
        { "depth_mm",        in.depth_mm },
        { "geometry",        geomTag },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "v_cutter";
    tooling.tool_dia_mm       = in.stroke_width_mm;
    tooling.tool_length_mm    = std::max(10.0, in.depth_mm * 5.0 + 5.0);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 800.0;
    tooling.feed_per_tooth_mm = 0.01;
    // Approximate stock removed: assume each glyph removes ~20-50% of the
    // glyph's bounding box (font_size² × depth).  Use 30% as a generic
    // estimate for the table's stencil glyphs.
    tooling.stock_removed_mm3 =
        0.30 * in.font_size_mm * in.font_size_mm * in.depth_mm *
        static_cast<double>(in.text.size());
    tooling.est_cycle_time_s  = std::max(1.0,
                                static_cast<double>(in.text.size()) * 0.7);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::engrave_text applied: '{}' glyphs={} spots={} geom={}",
                  in.text, glyphsBuilt, spotFallbacks, geomTag);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Without font-aware analysis, recognize() can only:
//   (a) Replay metadata signatures (full confidence).
//   (b) Heuristically detect rows of small shallow cylindrical features
//       (low confidence; only matches the spot-cut fallback path).

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

    // (b) Heuristic: collect small shallow cylinders (only matches the
    //     spot-cut fallback).  Real glyph extrusions leave planar faces,
    //     not cylindrical ones — recognition for those would require
    //     contour reconstruction which is out of scope.
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
        if (r * 2.0 >= 1.0) continue;
        const double vSpan = surf.LastVParameter() - surf.FirstVParameter();
        if (vSpan <= 0.0 || vSpan > 0.5) continue;
        spots.push_back({ fIdx, r, vSpan, cyl.Axis().Location() });
    }
    if (spots.size() < 2) return out;

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
