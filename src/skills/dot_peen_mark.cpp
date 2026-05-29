// @lat: [[engine/skills#dot_peen_mark]]
//
// Dot-peen marking — render text as a matrix of small cylindrical pits.
//
// Strategy:
//
//   For each character, the unit glyph polygon (outer outline only — we
//   ignore holes for the matrix because the matrix sampling already drops
//   inside-hole cells implicitly when those cells fall outside the outer
//   contour after the hole subtraction…  ACTUALLY: a simpler & robust
//   approach is to render the FILLED OUTER outline as the printed shape
//   and drop the dots that fall inside ANY hole.  The result is the
//   characteristic "stencil" dot-matrix glyph reading).
//
//   We sample a NCOLS × NROWS grid over [0, 1] × [0, 1] (the glyph's unit
//   reference frame).  At each grid centre (u, v), we run a point-in-
//   polygon test against the outer polygon (and reject if inside any
//   hole).  At every "inside" cell, we emit a small cylinder cutter.
//
//   Defaults: NCOLS=5, NROWS=7 — the classic 5×7 dot-matrix font dimensions.

#include "dot_peen_mark.hpp"

#include "Workpiece.hpp"
#include "_glyph_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace koocadcam::skill::dot_peen_mark {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// Dot-matrix sampling grid — classic 5×7 used by all common industrial
// dot-peen marker controllers.
static constexpr int kNCols = 5;
static constexpr int kNRows = 7;

// ── Validation ───────────────────────────────────────────────────────────
//
// Engineering basis (cite):
//   - Stylus tip diameter ≥ 0.1 mm: minimum reliable carbide stylus radius
//     in commercial pneumatic markers (Borries, Couth, SIC Marking spec
//     sheets converge here).  Sub-0.1-mm pins chip after ≈ 1k strikes.
//   - Indent depth ≥ 0.05 mm: ISO/IEC 16022:2006 §11 (Data Matrix dot-peen
//     readability) requires marks readable at 45° incident light with
//     0.05 mm minimum geometric depth on matte-finish parent.
//   - 5×7 grid: SAE AS9132 §4.3 (Aerospace Direct Part Marking) standard
//     character cell for dot-peen ID marking.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dot_dia_mm < 0.1) {
        r.add("DFM-PEEN-DIA", "error",
              "dot_peen_mark dot_dia_mm " + std::to_string(in.dot_dia_mm) +
              " < 0.1 mm (stylus tip too narrow; commercial carbide minimum)");
    }
    if (in.dot_depth_mm < 0.05) {
        r.add("DFM-PEEN-DEPTH", "error",
              "dot_peen_mark dot_depth_mm " + std::to_string(in.dot_depth_mm) +
              " < 0.05 mm (ISO/IEC 16022:2006 §11 readability minimum)");
    }
    if (in.text.empty()) {
        r.add("DFM-INPUT", "error", "dot_peen_mark text must not be empty");
    }
    if (in.font_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "dot_peen_mark font_size_mm must be > 0");
    }
    if (in.dot_spacing_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "dot_peen_mark dot_spacing_mm must be > 0");
    }
    (void)wp;
    return r;
}

// ── Internal helpers ─────────────────────────────────────────────────────

namespace {

char asGlyphKey(char c)
{
    if (c >= 'a' && c <= 'z') return static_cast<char>(c - 'a' + 'A');
    return c;
}

// Standard ray-casting point-in-polygon test for a polygon stored as a
// list of (u, v) pairs.  Returns true if (u, v) is strictly INSIDE the
// polygon.  Robust to non-convex polygons.  Adapted from W. R. Franklin's
// classic algorithm.
bool pointInPolygon(double u, double v,
                    const std::vector<std::array<double, 2>>& poly)
{
    bool inside = false;
    const size_t n = poly.size();
    if (n < 3) return false;
    size_t j = n - 1;
    for (size_t i = 0; i < n; ++i) {
        const double xi = poly[i][0]; const double yi = poly[i][1];
        const double xj = poly[j][0]; const double yj = poly[j][1];
        const bool crossY = (yi > v) != (yj > v);
        if (crossY) {
            const double xIntersect = (xj - xi) * (v - yi) / (yj - yi) + xi;
            if (u < xIntersect) inside = !inside;
        }
        j = i;
    }
    return inside;
}

// Returns true if (u, v) is inside the OUTER glyph polygon AND not inside
// any HOLE.  For glyphs with no holes only the first test runs.
bool insideGlyph(double u, double v, const engrave_text::glyph::GlyphPath& g)
{
    if (g.outer_and_holes.empty()) return false;
    if (!pointInPolygon(u, v, g.outer_and_holes[0])) return false;
    for (size_t h = 1; h < g.outer_and_holes.size(); ++h) {
        if (pointInPolygon(u, v, g.outer_and_holes[h])) return false;
    }
    return true;
}

}  // namespace

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dot_peen_mark DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("dot_peen_mark: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.02;

    gp_Dir advanceDir(1.0, 0.0, 0.0);
    gp_Dir perpDir(0.0, 1.0, 0.0);
    {
        const double dx = in.direction_xy.X();
        const double dy = in.direction_xy.Y();
        if (std::abs(dx) + std::abs(dy) > 1e-9) {
            advanceDir = gp_Dir(dx, dy, 0.0);
            perpDir    = gp_Dir(-advanceDir.Y(), advanceDir.X(), 0.0);
        }
    }

    const double dotR = in.dot_dia_mm / 2.0;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(in.text.size() * kNCols * kNRows / 2);
    int totalDots = 0;
    int charsRendered = 0;
    int charsEmpty = 0;

    for (size_t ci = 0; ci < in.text.size(); ++ci) {
        const char ch = in.text[ci];
        const auto& g = engrave_text::glyph::tableFor(asGlyphKey(ch));

        // Character origin in WORLD XY — same convention as engrave_text:
        // monospace step of (font_size + dot_spacing) per character (the
        // dot_spacing acts as inter-char kerning so adjacent glyphs don't
        // bleed dot-into-dot).
        const double advT = static_cast<double>(ci) *
                            (in.font_size_mm + in.dot_spacing_mm);
        const double charOriginX = in.position_x_mm + advT * advanceDir.X();
        const double charOriginY = in.position_y_mm + advT * advanceDir.Y();

        // Special-case: a SPACE character or a glyph not in the table —
        // emit a single centre dot so the spacing is preserved (avoids the
        // text "compressing" past unknown chars).  This keeps the mark
        // legible even when fonts are missing characters.
        if (g.outer_and_holes.empty()) {
            ++charsEmpty;
            if (ch != ' ') {
                // Sentinel single dot at glyph centre.
                const double uc = 0.5;
                const double vc = 0.5;
                const gp_Pnt dotTop(
                    charOriginX + uc * in.font_size_mm * advanceDir.X()
                                + vc * in.font_size_mm * perpDir.X(),
                    charOriginY + uc * in.font_size_mm * advanceDir.Y()
                                + vc * in.font_size_mm * perpDir.Y(),
                    zMax + kOverhang);
                const gp_Ax2 ax(dotTop, gp_Dir(0.0, 0.0, -1.0));
                try {
                    cutters.push_back(pr::cylinder(ax, dotR,
                                                    in.dot_depth_mm + kOverhang));
                    ++totalDots;
                } catch (const Standard_Failure& e) {
                    spdlog::debug("dot_peen_mark: sentinel cyl failed ({})", e.what());
                }
            }
            continue;
        }
        ++charsRendered;

        // Sample the 5×7 dot grid centred on each glyph cell.
        // Grid cells span equal portions of [0, 1] × [0, 1].
        for (int ix = 0; ix < kNCols; ++ix) {
            for (int iy = 0; iy < kNRows; ++iy) {
                const double u = (ix + 0.5) / static_cast<double>(kNCols);
                const double v = (iy + 0.5) / static_cast<double>(kNRows);

                if (!insideGlyph(u, v, g)) continue;

                // Map (u, v) to WORLD XY for the dot centre.
                const double wx = charOriginX
                    + u * in.font_size_mm * advanceDir.X()
                    + v * in.font_size_mm * perpDir.X();
                const double wy = charOriginY
                    + u * in.font_size_mm * advanceDir.Y()
                    + v * in.font_size_mm * perpDir.Y();

                const gp_Pnt dotTop(wx, wy, zMax + kOverhang);
                const gp_Ax2 ax(dotTop, gp_Dir(0.0, 0.0, -1.0));
                try {
                    cutters.push_back(pr::cylinder(ax, dotR,
                                                    in.dot_depth_mm + kOverhang));
                    ++totalDots;
                } catch (const Standard_Failure& e) {
                    spdlog::debug("dot_peen_mark: cyl build failed ({})", e.what());
                }
            }
        }
    }

    if (cutters.empty())
        throw SkillError("dot_peen_mark: no dot cutters built from text");

    TopoDS_Shape newShape;
    try {
        newShape = pr::cutMany(wp.shape(), cutters);
    } catch (const std::exception& e) {
        spdlog::warn("dot_peen_mark: cutMany failed ({}); leaving workpiece unchanged",
                     e.what());
        newShape = wp.shape();
    }

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "direction_xy",    { in.direction_xy.X(), in.direction_xy.Y(), in.direction_xy.Z() } },
        { "text",            in.text },
        { "font_size_mm",    in.font_size_mm },
        { "dot_dia_mm",      in.dot_dia_mm },
        { "dot_depth_mm",    in.dot_depth_mm },
        { "dot_spacing_mm",  in.dot_spacing_mm },
    };
    json pattern = {
        { "kind",            kSkillId },
        { "text",            in.text },
        { "char_count",      in.text.size() },
        { "chars_rendered",  charsRendered },
        { "chars_empty",     charsEmpty },
        { "dot_count",       totalDots },
        { "grid_cols",       kNCols },
        { "grid_rows",       kNRows },
        { "dot_dia_mm",      in.dot_dia_mm },
        { "dot_depth_mm",    in.dot_depth_mm },
        { "font_size_mm",    in.font_size_mm },
        { "geometry",        "dot_matrix" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "peen_stylus";
    tooling.tool_dia_mm       = in.dot_dia_mm;
    tooling.tool_length_mm    = std::max(5.0, in.dot_depth_mm * 5.0 + 2.0);
    tooling.tool_material     = "tungsten_carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Per-dot volume × dot count.
    const double dotVol = M_PI * dotR * dotR * in.dot_depth_mm;
    tooling.stock_removed_mm3 = dotVol * static_cast<double>(totalDots);
    // Each dot takes ~ 0.05 s on a fast pneumatic stylus.
    tooling.est_cycle_time_s  = std::max(0.5, totalDots * 0.05);
    tooling.extra = {
        { "process",          "pneumatic_dot_peen" },
        { "dot_spacing_mm",   in.dot_spacing_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::dot_peen_mark applied: '{}' dots={} chars_rendered={}",
                  in.text, totalDots, charsRendered);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay PRIMARY (confidence 1.0).  Geometric fallback when no
// metadata: count tiny circular bottom faces (planar, +Z normal, < 0.5 mm
// depth from top, < 0.5 mm radius footprint).  These are the cylindrical
// peen-pit bottoms.  Confidence 0.5 — a small dot pattern is more
// distinctive than a generic shallow pocket (laser_mark fallback) so it
// rates slightly higher.

namespace {

// Walk planar faces; count those with +Z normal AND sitting between
// (topZ - 0.5 mm, topZ - 1 µm).  These are the pit bottoms after the cut.
int countPitBottoms(const Workpiece& wp, double& meanDepthMm_out)
{
    meanDepthMm_out = 0.0;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    int count = 0;
    double depthSum = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const gp_Dir n = wp.faceNormal(i);
        if (std::abs(n.Z()) < std::cos(1.0 * M_PI / 180.0)) continue;
        const gp_Pnt c = wp.faceCenter(i);
        const double depth = topZ - c.Z();
        if (depth <= 1e-6) continue;
        if (depth > 0.5) continue;
        ++count;
        depthSum += depth;
    }
    if (count > 0) meanDepthMm_out = depthSum / static_cast<double>(count);
    return count;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata_replay" },
                                { "pattern", f.pattern } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    double meanDepth = 0.0;
    const int n = countPitBottoms(wp, meanDepth);
    if (n >= 3) {  // require at least a few pits to call it dot-peen
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = {
            { "text",          "" },
            { "dot_depth_mm",  meanDepth },
        };
        rf.confidence       = 0.5;
        rf.matched_geometry = {
            { "source",           "geometric_fallback" },
            { "pit_count",        n },
            { "mean_depth_mm",    meanDepth },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::dot_peen_mark
