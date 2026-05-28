#pragma once
// @lat: [[engine/skills#engrave_text — glyph table]]
//
// Hard-coded minimal glyph table for `engrave_text`.  Each glyph is a
// `GlyphPath` — a list of one or more closed 2-D polygons in a UNIT-SIZE
// reference frame (x ∈ [0, 1], y ∈ [0, 1], baseline at y = 0, ascender
// at y = 1).  The first polygon is the OUTER outline; any additional
// polygons are INNER holes (e.g. the bowls of "O", "B", "D", "A").
//
// Coverage: digits 0-9 + uppercase A-Z (36 glyphs total).  Unknown chars
// fall back to a default-empty glyph (callers should then use the legacy
// "one tiny cylinder per character" spot-cut approximation — see
// `engrave_text::apply()`).
//
// Coordinate convention (per Z-orientation of the engraved face):
//   The polygons are 2-D in the (u, v) parameter space of the engrave
//   plane.  `engrave_text::apply()` maps (u, v) → world by anchoring at
//   (position_x_mm + i·advance, position_y_mm) and scaling by font_size_mm.
//   The polygons are CCW-wound when viewed from OUTSIDE the workpiece
//   (the side the V-cutter approaches from).
//
// Design philosophy:
//   These glyphs are deliberately blocky / "stencil" letters with axis-
//   aligned strokes — they are the SIMPLEST closed polygons that read as
//   the letter at small font sizes (≤ 5 mm) common in engraving.  They are
//   NOT meant to look like a typographic font — that would require Bézier
//   contours and a real font hinter (FreeType).  For real fonts, swap the
//   table builder for a FreeType-backed loader.

#include <array>
#include <string>
#include <vector>

namespace koocadcam::skill::engrave_text::glyph {

struct GlyphPath {
    // `outer_and_holes[0]` is the outer outline; subsequent vectors are inner
    // holes (used by "O", "0", "B", "D", "P", "Q", "R", "A", "4", "6", "8", "9").
    std::vector<std::vector<std::array<double, 2>>> outer_and_holes;
};

// Implemented in this header as an inline function (header-only minimal
// dep — the entire table fits in ~200 lines of literal coordinates).
inline const GlyphPath& tableFor(char c)
{
    // Stroke width in unit coords — about 1/5 of glyph height.  Bowl
    // openings on letters like "O" use a roughly same-thickness ring.
    static const double s = 0.20;       // stroke half-width = 0.10

    // ---- helpers for the static initializer ----
    auto rect = [](double x0, double y0, double x1, double y1)
        -> std::vector<std::array<double, 2>>
    {
        // CCW.  (x0, y0)-(x1, y1) is the axis-aligned bounding box.
        return { {{x0, y0}}, {{x1, y0}}, {{x1, y1}}, {{x0, y1}} };
    };

    // ── Static table: 36 glyphs, built once. ─────────────────────────────
    // Lambda captures `rect` + `s` from enclosing scope; safe because this
    // lambda is invoked exactly once via the trailing `table = buildTable()`
    // (line ~766) during static init of `table`, so reference lifetimes are
    // valid during that single invocation.
    static const auto buildTable = [&]() {
        std::vector<GlyphPath> t(128);   // index by ASCII code

        // Generic "block letter" template — a hollow rectangle with a hole.
        // outer: (0,0)→(1,1); inner hole: (s,s)→(1-s,1-s).  Used by O, 0, D.
        auto blockHollow = [&]() {
            GlyphPath g;
            g.outer_and_holes.push_back(rect(0.0, 0.0, 1.0, 1.0));
            // Inner hole — wound CW (= reversed CCW) to indicate a HOLE in
            // most CAD systems.  We just store them; the caller is
            // responsible for using BRepBuilderAPI_MakeFace with the wire
            // marked as a hole.
            std::vector<std::array<double, 2>> hole = {
                {{s,     s    }},
                {{1 - s, s    }},
                {{1 - s, 1 - s}},
                {{s,     1 - s}},
            };
            g.outer_and_holes.push_back(hole);
            return g;
        };

        // Solid rectangle (no hole).
        auto solidRect = [&](double x0, double y0, double x1, double y1) {
            GlyphPath g;
            g.outer_and_holes.push_back(rect(x0, y0, x1, y1));
            return g;
        };

        // Multi-rect glyph — outer outline traced as a single polygon.  The
        // builder takes a list of rectangles, ORs them together visually,
        // and returns a single CCW polygon outline.  We do this with a hand-
        // traced outline per letter rather than a general polygon-union.
        // For simplicity many letters are built as a SINGLE thick outer
        // outline whose interior is one piece (no holes).  See per-letter
        // comments below.

        // ===== Digits =====

        // "0" — like "O", hollow rect.
        t['0'] = blockHollow();

        // "1" — single vertical bar centred.
        t['1'] = solidRect(0.5 - s / 2, 0.0, 0.5 + s / 2, 1.0);

        // "2" — S-shape laid out as a single CCW outline.  We trace a
        // stylized "2" with stroke width s.
        {
            GlyphPath g;
            // Trace clockwise vertices going CCW: start bottom-left, top
            // along bottom bar, up the right side a bit, across left to
            // the right side of the upper bar, up, across the top, down
            // outer right, around …  For simplicity use a stair-step
            // approximation:
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       s      }},
                {{s,         s      }},
                {{s,         0.5 - s/2}},
                {{1.0,       0.5 - s/2}},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
                {{0.0,       1.0 - s}},
                {{1.0 - s,   1.0 - s}},
                {{1.0 - s,   0.5 + s/2}},
                {{0.0,       0.5 + s/2}},
            };
            g.outer_and_holes.push_back(outer);
            t['2'] = g;
        }

        // "3" — like "E" mirrored, no left bar.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
                {{0.0,       1.0 - s}},
                {{1.0 - s,   1.0 - s}},
                {{1.0 - s,   0.5 + s/2}},
                {{0.0,       0.5 + s/2}},
                {{0.0,       0.5 - s/2}},
                {{1.0 - s,   0.5 - s/2}},
                {{1.0 - s,   s      }},
                {{0.0,       s      }},
            };
            g.outer_and_holes.push_back(outer);
            t['3'] = g;
        }

        // "4" — L-shape mirrored with horizontal mid bar.  Has a hole.
        {
            GlyphPath g;
            // Outer: rectangle.  Hole: upper-left and lower-right cutouts
            // create the "4" shape.
            // For simplicity: vertical right bar full height + horizontal
            // mid bar + short upper-left vertical.  Trace a single outer
            // outline:
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.5 - s/2}},     // start at left edge of mid bar
                {{1.0 - s,   0.5 - s/2}},     // right along bottom of mid bar
                {{1.0 - s,   0.0      }},     // down to baseline
                {{1.0,       0.0      }},
                {{1.0,       1.0      }},
                {{1.0 - s,   1.0      }},
                {{1.0 - s,   0.5 + s/2}},
                {{0.0,       0.5 + s/2}},     // left along top of mid bar
                {{0.0,       1.0      }},     // up the left edge
                // Wait — we need an "L" pointing into the upper region,
                // so the upper-left vertical extends from mid bar to top.
                // The outline should turn here, not loop.  Let me rebuild
                // as two disjoint rects merged into one shape (the mid bar
                // bridges them).
            };
            // Rebuild as: outer outline of the union of three rects:
            //   R1: right vertical bar       (1-s, 0)  → (1, 1)
            //   R2: horizontal mid bar       (0, 0.5-s/2) → (1, 0.5+s/2)
            //   R3: upper-left vertical bar  (0, 0.5+s/2) → (s, 1)
            // Their union is a connected polygon:
            outer = {
                {{1.0 - s,   0.0      }},
                {{1.0,       0.0      }},
                {{1.0,       1.0      }},
                {{1.0 - s,   1.0      }},
                {{1.0 - s,   0.5 + s/2}},
                {{s,         0.5 + s/2}},
                {{s,         1.0      }},
                {{0.0,       1.0      }},
                {{0.0,       0.5 - s/2}},
                {{1.0 - s,   0.5 - s/2}},
            };
            g.outer_and_holes.push_back(outer);
            t['4'] = g;
        }

        // "5" — like "2" but flipped (top bar, then right side, then mid,
        //  then left side, then bottom).
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       0.5 + s/2}},
                {{s,         0.5 + s/2}},
                {{s,         1.0 - s}},
                {{1.0,       1.0 - s}},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
                {{0.0,       0.5 - s/2}},
                {{1.0 - s,   0.5 - s/2}},
                {{1.0 - s,   s      }},
                {{0.0,       s      }},
            };
            g.outer_and_holes.push_back(outer);
            t['5'] = g;
        }

        // "6" — like "G" without the right horizontal at bottom.  Has a hole.
        {
            GlyphPath g;
            // Outer: full rectangle.  Hole: upper-right pocket above the
            // mid bar.  Trace a single outer outline that snakes around
            // the upper-right cutout, ending in a closed loop.
            std::vector<std::array<double, 2>> outer = {
                {{0.0,     0.0      }},
                {{1.0,     0.0      }},
                {{1.0,     0.5 + s/2}},
                {{s,       0.5 + s/2}},
                {{s,       1.0 - s  }},
                {{1.0,     1.0 - s  }},
                {{1.0,     1.0      }},
                {{0.0,     1.0      }},
            };
            g.outer_and_holes.push_back(outer);
            // Inner hole: the rectangular pocket in the lower bowl.
            std::vector<std::array<double, 2>> hole = {
                {{s,       s      }},
                {{1.0 - s, s      }},
                {{1.0 - s, 0.5 - s/2}},
                {{s,       0.5 - s/2}},
            };
            g.outer_and_holes.push_back(hole);
            t['6'] = g;
        }

        // "7" — top bar + diagonal stroke.  Stylized as top bar + right
        // vertical only (simpler — looks like a flattened "7").
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       1.0 - s}},
                {{1.0 - s,   1.0 - s}},
                {{1.0 - s,   0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['7'] = g;
        }

        // "8" — two stacked hollow rects.  Use one outer rect with two holes.
        {
            GlyphPath g;
            g.outer_and_holes.push_back(rect(0.0, 0.0, 1.0, 1.0));
            // Upper hole.
            g.outer_and_holes.push_back({
                {{s,       0.5 + s/2}},
                {{1.0 - s, 0.5 + s/2}},
                {{1.0 - s, 1.0 - s  }},
                {{s,       1.0 - s  }},
            });
            // Lower hole.
            g.outer_and_holes.push_back({
                {{s,       s        }},
                {{1.0 - s, s        }},
                {{1.0 - s, 0.5 - s/2}},
                {{s,       0.5 - s/2}},
            });
            t['8'] = g;
        }

        // "9" — like "6" upside-down.  Hole in upper bowl, no lower-left
        // open mouth.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{1.0 - s, 0.0      }},
                {{1.0,     0.0      }},
                {{1.0,     1.0      }},
                {{0.0,     1.0      }},
                {{0.0,     0.5 - s/2}},
                {{1.0 - s, 0.5 - s/2}},
            };
            g.outer_and_holes.push_back(outer);
            std::vector<std::array<double, 2>> hole = {
                {{s,       0.5 + s/2}},
                {{1.0 - s, 0.5 + s/2}},
                {{1.0 - s, 1.0 - s  }},
                {{s,       1.0 - s  }},
            };
            g.outer_and_holes.push_back(hole);
            t['9'] = g;
        }

        // ===== Letters =====

        // "A" — peaked outer + triangle hole.  Approximated as: trapezoid
        // outer + rectangular hole.  For axis-aligned simplicity we make
        // it a rectangle outer + hole.
        {
            GlyphPath g;
            g.outer_and_holes.push_back(rect(0.0, 0.0, 1.0, 1.0));
            std::vector<std::array<double, 2>> hole = {
                {{s,       s        }},
                {{1.0 - s, s        }},
                {{1.0 - s, 0.5 - s/2}},
                {{s,       0.5 - s/2}},
            };
            g.outer_and_holes.push_back(hole);
            // Also need upper-hole-with-no-bar (the apex of "A").  Use a
            // single hole above the crossbar.
            std::vector<std::array<double, 2>> upperHole = {
                {{s,       0.5 + s/2}},
                {{1.0 - s, 0.5 + s/2}},
                {{1.0 - s, 1.0 - s  }},
                {{s,       1.0 - s  }},
            };
            g.outer_and_holes.push_back(upperHole);
            t['A'] = g;
        }

        // "B" — vertical bar + two bowls (two holes).
        {
            GlyphPath g;
            g.outer_and_holes.push_back(rect(0.0, 0.0, 1.0, 1.0));
            g.outer_and_holes.push_back({
                {{s,       s        }},
                {{1.0 - s, s        }},
                {{1.0 - s, 0.5 - s/2}},
                {{s,       0.5 - s/2}},
            });
            g.outer_and_holes.push_back({
                {{s,       0.5 + s/2}},
                {{1.0 - s, 0.5 + s/2}},
                {{1.0 - s, 1.0 - s  }},
                {{s,       1.0 - s  }},
            });
            t['B'] = g;
        }

        // "C" — like a frame open on the right.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,     0.0    }},
                {{1.0,     0.0    }},
                {{1.0,     s      }},
                {{s,       s      }},
                {{s,       1.0 - s}},
                {{1.0,     1.0 - s}},
                {{1.0,     1.0    }},
                {{0.0,     1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['C'] = g;
        }

        // "D" — hollow rect (like "O").
        t['D'] = blockHollow();

        // "E" — three horizontal bars + left vertical.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       s      }},
                {{s,         s      }},
                {{s,         0.5 - s/2}},
                {{1.0,       0.5 - s/2}},
                {{1.0,       0.5 + s/2}},
                {{s,         0.5 + s/2}},
                {{s,         1.0 - s}},
                {{1.0,       1.0 - s}},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['E'] = g;
        }

        // "F" — like "E" missing the bottom bar.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{s,         0.0    }},
                {{s,         0.5 - s/2}},
                {{1.0,       0.5 - s/2}},
                {{1.0,       0.5 + s/2}},
                {{s,         0.5 + s/2}},
                {{s,         1.0 - s}},
                {{1.0,       1.0 - s}},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['F'] = g;
        }

        // "G" — like "C" with a right-side mid bar.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       0.5 + s/2}},
                {{0.5,       0.5 + s/2}},
                {{0.5,       0.5 - s/2}},
                {{1.0 - s,   0.5 - s/2}},
                {{1.0 - s,   s      }},
                {{s,         s      }},
                {{s,         1.0 - s}},
                {{1.0,       1.0 - s}},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['G'] = g;
        }

        // "H" — two verticals + mid bar.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{s,         0.0    }},
                {{s,         0.5 - s/2}},
                {{1.0 - s,   0.5 - s/2}},
                {{1.0 - s,   0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       1.0    }},
                {{1.0 - s,   1.0    }},
                {{1.0 - s,   0.5 + s/2}},
                {{s,         0.5 + s/2}},
                {{s,         1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['H'] = g;
        }

        // "I" — single vertical bar.
        t['I'] = solidRect(0.5 - s/2, 0.0, 0.5 + s/2, 1.0);

        // "J" — right vertical + bottom curl (just a horizontal at bottom).
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       1.0    }},
                {{1.0 - s,   1.0    }},
                {{1.0 - s,   s      }},
                {{0.0,       s      }},
            };
            g.outer_and_holes.push_back(outer);
            t['J'] = g;
        }

        // "K" — left vertical + two diagonals (approximated as horizontals).
        {
            GlyphPath g;
            // We simplify K as: left vertical bar + mid-height horizontal
            // bar to the right (so the "arms" of K become a single
            // horizontal stroke meeting the vertical at mid-height).
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{s,         0.0    }},
                {{s,         0.5 - s/2}},
                {{1.0,       0.5 - s/2}},
                {{1.0,       0.5 + s/2}},
                {{s,         0.5 + s/2}},
                {{s,         1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['K'] = g;
        }

        // "L" — left vertical + bottom horizontal.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       s      }},
                {{s,         s      }},
                {{s,         1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['L'] = g;
        }

        // "M" — two verticals + center "V" approximated as center vertical
        // dropping from top to mid.
        {
            GlyphPath g;
            // L vertical | center vertical (top half) | R vertical, with
            // top bar bridging them.
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{s,         0.0    }},
                {{s,         1.0 - s}},
                {{0.5 - s/2, 1.0 - s}},
                {{0.5 - s/2, 0.5    }},
                {{0.5 + s/2, 0.5    }},
                {{0.5 + s/2, 1.0 - s}},
                {{1.0 - s,   1.0 - s}},
                {{1.0 - s,   0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['M'] = g;
        }

        // "N" — two verticals (no diagonal — approximated as two solid bars
        //  with a top connector).
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{s,         0.0    }},
                {{s,         1.0 - s}},
                {{1.0 - s,   1.0 - s}},
                {{1.0 - s,   0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['N'] = g;
        }

        // "O" — hollow rect.
        t['O'] = blockHollow();

        // "P" — vertical + upper bowl (one hole in upper half).
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{s,         0.0    }},
                {{s,         0.5 - s/2}},
                {{1.0,       0.5 - s/2}},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            std::vector<std::array<double, 2>> hole = {
                {{s,       0.5 + s/2}},
                {{1.0 - s, 0.5 + s/2}},
                {{1.0 - s, 1.0 - s  }},
                {{s,       1.0 - s  }},
            };
            g.outer_and_holes.push_back(hole);
            t['P'] = g;
        }

        // "Q" — like "O" with a tail.  Approximated as a hollow rect
        // (= "O") for simplicity.
        t['Q'] = blockHollow();

        // "R" — like "P" plus a right diagonal (approximated as right
        // vertical extending below the bowl).
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{s,         0.0    }},
                {{s,         0.5 - s/2}},
                {{1.0 - s,   0.5 - s/2}},
                {{1.0 - s,   0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            std::vector<std::array<double, 2>> hole = {
                {{s,       0.5 + s/2}},
                {{1.0 - s, 0.5 + s/2}},
                {{1.0 - s, 1.0 - s  }},
                {{s,       1.0 - s  }},
            };
            g.outer_and_holes.push_back(hole);
            t['R'] = g;
        }

        // "S" — S-curve, approximated as "5" mirrored vertically (= "2"
        //  shape, actually).  Reuse "2".
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       s      }},
                {{s,         s      }},
                {{s,         0.5 - s/2}},
                {{1.0,       0.5 - s/2}},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
                {{0.0,       1.0 - s}},
                {{1.0 - s,   1.0 - s}},
                {{1.0 - s,   0.5 + s/2}},
                {{0.0,       0.5 + s/2}},
            };
            g.outer_and_holes.push_back(outer);
            t['S'] = g;
        }

        // "T" — top bar + center vertical.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.5 - s/2, 0.0    }},
                {{0.5 + s/2, 0.0    }},
                {{0.5 + s/2, 1.0 - s}},
                {{1.0,       1.0 - s}},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
                {{0.0,       1.0 - s}},
                {{0.5 - s/2, 1.0 - s}},
            };
            g.outer_and_holes.push_back(outer);
            t['T'] = g;
        }

        // "U" — left + right verticals + bottom bar.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       1.0    }},
                {{1.0 - s,   1.0    }},
                {{1.0 - s,   s      }},
                {{s,         s      }},
                {{s,         1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['U'] = g;
        }

        // "V" — diagonals.  Approximated as a stripped "U" (no closing
        //  bottom — but that would be an open polygon).  Instead use a
        //  trapezoid outline narrowing at the bottom.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.5 - s/2, 0.0    }},
                {{0.5 + s/2, 0.0    }},
                {{1.0,       1.0    }},
                {{1.0 - s,   1.0    }},
                {{0.5,       2 * s  }},
                {{s,         1.0    }},
                {{0.0,       1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['V'] = g;
        }

        // "W" — like two V's.  Approximated as a similar trapezoid family.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       1.0    }},
                {{s,         0.0    }},
                {{0.25 + s/2, 0.0   }},
                {{0.4,       0.7    }},
                {{0.5,       0.4    }},
                {{0.6,       0.7    }},
                {{0.75 - s/2, 0.0   }},
                {{1.0 - s,   0.0    }},
                {{1.0,       1.0    }},
                {{1.0 - s,   1.0    }},
                {{0.8,       0.3    }},
                {{0.65,      0.8    }},
                {{0.5,       0.5    }},
                {{0.35,      0.8    }},
                {{0.2,       0.3    }},
                {{s,         1.0    }},
            };
            g.outer_and_holes.push_back(outer);
            t['W'] = g;
        }

        // "X" — diagonal cross.  Approximated as horizontal mid bar (since
        //  axis-aligned approximation), forming a simple "+"
        //  shape.  Better: trace a + outline.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.5 - s/2, 0.0      }},
                {{0.5 + s/2, 0.0      }},
                {{0.5 + s/2, 0.5 - s/2}},
                {{1.0,       0.5 - s/2}},
                {{1.0,       0.5 + s/2}},
                {{0.5 + s/2, 0.5 + s/2}},
                {{0.5 + s/2, 1.0      }},
                {{0.5 - s/2, 1.0      }},
                {{0.5 - s/2, 0.5 + s/2}},
                {{0.0,       0.5 + s/2}},
                {{0.0,       0.5 - s/2}},
                {{0.5 - s/2, 0.5 - s/2}},
            };
            g.outer_and_holes.push_back(outer);
            t['X'] = g;
        }

        // "Y" — like T but with a short stem only in the lower half.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.5 - s/2, 0.0      }},
                {{0.5 + s/2, 0.0      }},
                {{0.5 + s/2, 0.5 + s/2}},
                {{1.0,       1.0      }},
                {{1.0 - s,   1.0      }},
                {{0.5,       0.6      }},
                {{s,         1.0      }},
                {{0.0,       1.0      }},
                {{0.5 - s/2, 0.5 + s/2}},
            };
            g.outer_and_holes.push_back(outer);
            t['Y'] = g;
        }

        // "Z" — like a "2" mirrored along the vertical axis.
        {
            GlyphPath g;
            std::vector<std::array<double, 2>> outer = {
                {{0.0,       0.0    }},
                {{1.0,       0.0    }},
                {{1.0,       s      }},
                {{s,         s      }},
                {{1.0,       1.0 - s}},
                {{1.0,       1.0    }},
                {{0.0,       1.0    }},
                {{0.0,       1.0 - s}},
                {{1.0 - s,   1.0 - s}},
                {{0.0,       s      }},
            };
            g.outer_and_holes.push_back(outer);
            t['Z'] = g;
        }

        return t;
    };

    static const auto table = buildTable();
    static const GlyphPath kEmpty;  // default-empty for unknown chars

    if (c < 0 || c >= 128) return kEmpty;
    const GlyphPath& g = table[static_cast<unsigned char>(c)];
    if (g.outer_and_holes.empty()) return kEmpty;
    return g;
}

}  // namespace koocadcam::skill::engrave_text::glyph
