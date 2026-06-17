// @lat: [[engine/reverse-route#compound-grammar]]

#include "CompoundGrammar.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

namespace koocadcam::re {

namespace {

using json = nlohmann::json;

// A physical drilled hole, assembled from one or more recognizer atoms.  A
// single hole can produce several atoms: drill_hole + drill_through_hole report
// the same face, and STEP can split one cylinder into several faces.  We merge
// them by location/axis/diameter and sum the wrap of DISTINCT faces — a real
// hole completes a revolution (~2π); a pocket-corner fillet is a lone ~90° arc.
struct Hole {
    double x, y, z;
    double dia;
    double ax, ay, az;            // axis direction
    double wrap = 0.0;            // summed wrap over distinct faces (radians)
    std::unordered_set<int> faces;
};

// A real drilled hole's faces span (close to) a full revolution.  A concave
// corner fillet of a rectangular pocket spans only ~π/2 — and four such corners
// are concyclic, so without this gate they masquerade as a 4-hole bolt circle.
constexpr double kHoleWrapMin = 1.5 * M_PI;     // 270°: full hole, not a fillet
constexpr double kColocTolMm  = 0.3;            // same hole if centres within this

double num(const json& p, const char* k)
{
    return (p.contains(k) && p[k].is_number()) ? p[k].get<double>() : 0.0;
}

bool axisParallel(double ax, double ay, double az,
                  double bx, double by, double bz)
{
    return std::abs(ax * bx + ay * by + az * bz) > 0.999;
}

// Two holes are COAXIAL when their axes are parallel AND the two axis lines are
// (near-)the-same line — the perpendicular distance between them is < tol.
// (Coaxial = same axis line; parallel-but-offset = a bolt-circle member.)
bool coaxialLines(double ax, double ay, double az, double aX, double aY, double aZ,
                  double bx, double by, double bz, double bX, double bY, double bZ)
{
    if (!axisParallel(ax, ay, az, bx, by, bz)) return false;
    const double wx = bX - aX, wy = bY - aY, wz = bZ - aZ;
    const double dot = wx * ax + wy * ay + wz * az;     // component along axis a
    const double px = wx - dot * ax, py = wy - dot * ay, pz = wz - dot * az;
    return std::sqrt(px * px + py * py + pz * pz) < 0.3;
}

// Algebraic (Kåsa) circle fit of XY points: minimise sum (x²+y²+Dx+Ey+F)².
// Returns false if the points are (near-)collinear / degenerate.
bool fitCircleXY(const std::vector<Hole>& pts, double& cx, double& cy,
                 double& r, double& maxResidual)
{
    const std::size_t n = pts.size();
    if (n < 3) return false;
    double Sx = 0, Sy = 0, Sxx = 0, Syy = 0, Sxy = 0;
    double Sxz = 0, Syz = 0, Sz = 0;   // z = x²+y²
    for (const auto& p : pts) {
        const double zi = p.x * p.x + p.y * p.y;
        Sx += p.x;  Sy += p.y;  Sz += zi;
        Sxx += p.x * p.x;  Syy += p.y * p.y;  Sxy += p.x * p.y;
        Sxz += p.x * zi;   Syz += p.y * zi;
    }
    const double N = static_cast<double>(n);
    // Solve the 3x3 system for (D, E, F) via Cramer's rule.
    //  [Sxx Sxy Sx ] [D]   [-Sxz]
    //  [Sxy Syy Sy ] [E] = [-Syz]
    //  [Sx  Sy  N  ] [F]   [-Sz ]
    const double det =
        Sxx * (Syy * N - Sy * Sy) - Sxy * (Sxy * N - Sy * Sx) +
        Sx  * (Sxy * Sy - Syy * Sx);
    if (std::abs(det) < 1e-9) return false;   // collinear / degenerate
    const double bx = -Sxz, by = -Syz, bz = -Sz;
    const double D =
        (bx * (Syy * N - Sy * Sy) - Sxy * (by * N - Sy * bz) +
         Sx  * (by * Sy - Syy * bz)) / det;
    const double E =
        (Sxx * (by * N - bz * Sy) - bx * (Sxy * N - Sy * Sx) +
         Sx  * (Sxy * bz - by * Sx)) / det;
    const double F =
        (Sxx * (Syy * bz - by * Sy) - Sxy * (Sxy * bz - by * Sx) +
         bx  * (Sxy * Sy - Syy * Sx)) / det;
    cx = -D / 2.0;
    cy = -E / 2.0;
    const double rsq = cx * cx + cy * cy - F;
    if (rsq <= 0.0) return false;
    r = std::sqrt(rsq);
    maxResidual = 0.0;
    for (const auto& p : pts) {
        const double d = std::hypot(p.x - cx, p.y - cy);
        maxResidual = std::max(maxResidual, std::abs(d - r));
    }
    return true;
}

// Total-least-squares (PCA) line fit of XY points.  Returns the unit direction
// (dirx,diry) and centroid (cx,cy), the max PERPENDICULAR residual, and each
// point's signed projection onto the direction (for pitch analysis).  Returns
// false only on a degenerate (single-location) set.
bool fitLineXY(const std::vector<Hole>& pts, double& dirx, double& diry,
               double& cx, double& cy, double& maxResidual,
               std::vector<double>& proj)
{
    const std::size_t n = pts.size();
    if (n < 3) return false;
    double mx = 0, my = 0;
    for (const auto& p : pts) { mx += p.x; my += p.y; }
    mx /= n;  my /= n;
    double Sxx = 0, Sxy = 0, Syy = 0;
    for (const auto& p : pts) {
        const double dx = p.x - mx, dy = p.y - my;
        Sxx += dx * dx;  Sxy += dx * dy;  Syy += dy * dy;
    }
    // Principal eigenvector of the 2x2 covariance [[Sxx,Sxy],[Sxy,Syy]].
    const double tr = Sxx + Syy, det = Sxx * Syy - Sxy * Sxy;
    const double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));
    const double l1 = tr / 2.0 + disc;          // larger eigenvalue
    double vx, vy;
    if (std::abs(Sxy) > 1e-12) { vx = l1 - Syy; vy = Sxy; }
    else if (Sxx >= Syy)       { vx = 1.0; vy = 0.0; }
    else                       { vx = 0.0; vy = 1.0; }
    const double vn = std::hypot(vx, vy);
    if (vn < 1e-12) return false;
    vx /= vn;  vy /= vn;
    dirx = vx;  diry = vy;  cx = mx;  cy = my;
    maxResidual = 0.0;
    proj.clear();  proj.reserve(n);
    for (const auto& p : pts) {
        const double dx = p.x - mx, dy = p.y - my;
        proj.push_back(dx * vx + dy * vy);              // along the line
        maxResidual = std::max(maxResidual, std::abs(dx * (-vy) + dy * vx));
    }
    return true;
}

// Fit a 2D lattice: every centre ≈ corner + i·u + j·v for integers i,j>=0,
// forming a FILLED cols×rows rectangle (cols,rows >= 2, cols*rows == n >= 6).
// Robust to a non-square pitch: u = nearest neighbour of a corner hole, v = the
// nearest neighbour whose perpendicular offset from u is real (not collinear).
// Returns the two pitches, the grid extents, and the max reconstruction
// residual.  A 2x2 grid (4 concyclic points) is intentionally NOT handled here —
// it is claimed upstream as a bolt circle.
bool fitGridXY(const std::vector<Hole>& pts, double& pitchU, double& pitchV,
               int& cols, int& rows, double& maxResidual,
               double& originX, double& originY,
               double& uDx, double& uDy, double& vDx, double& vDy)
{
    const std::size_t n = pts.size();
    if (n < 6) return false;
    // Corner seed: smallest x, then smallest y.
    std::size_t c0 = 0;
    for (std::size_t k = 1; k < n; ++k)
        if (pts[k].x < pts[c0].x - 1e-9 ||
            (std::abs(pts[k].x - pts[c0].x) <= 1e-9 && pts[k].y < pts[c0].y))
            c0 = k;
    const double Ox = pts[c0].x, Oy = pts[c0].y;
    // u = nearest neighbour of the corner.
    std::size_t un = n;  double ubest = 1e300;
    for (std::size_t k = 0; k < n; ++k) {
        if (k == c0) continue;
        const double dd = std::hypot(pts[k].x - Ox, pts[k].y - Oy);
        if (dd < ubest) { ubest = dd; un = k; }
    }
    if (un == n) return false;
    const double ux = pts[un].x - Ox, uy = pts[un].y - Oy;
    const double ulen = std::hypot(ux, uy);
    if (ulen < 1e-9) return false;
    // v = nearest neighbour whose perpendicular distance to the u-axis is real.
    std::size_t vn = n;  double vbest = 1e300;
    for (std::size_t k = 0; k < n; ++k) {
        if (k == c0 || k == un) continue;
        const double wx = pts[k].x - Ox, wy = pts[k].y - Oy;
        const double perp = std::abs(ux * wy - uy * wx) / ulen;
        if (perp < 0.3) continue;                       // collinear with u
        const double dd = std::hypot(wx, wy);
        if (dd < vbest) { vbest = dd; vn = k; }
    }
    if (vn == n) return false;
    const double vx = pts[vn].x - Ox, vy = pts[vn].y - Oy;
    const double cross = ux * vy - uy * vx;
    if (std::abs(cross) < 1e-6) return false;           // degenerate basis
    // Reconstruct integer (i,j) for every point via Cramer's rule.
    int maxi = 0, maxj = 0;
    maxResidual = 0.0;
    std::set<std::pair<int, int>> cells;
    for (std::size_t k = 0; k < n; ++k) {
        const double px = pts[k].x - Ox, py = pts[k].y - Oy;
        const long ii = std::lround((px * vy - py * vx) / cross);
        const long jj = std::lround((ux * py - uy * px) / cross);
        if (ii < 0 || jj < 0) return false;
        const double rx = px - (ux * ii + vx * jj);
        const double ry = py - (uy * ii + vy * jj);
        maxResidual = std::max(maxResidual, std::hypot(rx, ry));
        cells.insert({ static_cast<int>(ii), static_cast<int>(jj) });
        maxi = std::max(maxi, static_cast<int>(ii));
        maxj = std::max(maxj, static_cast<int>(jj));
    }
    if (maxResidual > 0.3) return false;
    cols = maxi + 1;  rows = maxj + 1;
    if (cols < 2 || rows < 2) return false;             // not 2-D
    if (cells.size() != n) return false;                // a cell claimed twice
    if (static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows) != n)
        return false;                                   // not a FULL rectangle
    pitchU = ulen;  pitchV = std::hypot(vx, vy);
    // Origin (corner) + unit basis directions, so the pattern can be regenerated.
    originX = Ox;  originY = Oy;
    uDx = ux / ulen;          uDy = uy / ulen;
    vDx = vx / pitchV;        vDy = vy / pitchV;
    return true;
}

// Emit a hole-pattern compound for one group of equal-diameter, parallel-axis,
// REVOLUTION-COMPLETE drilled holes.  Three declared patterns, tried in order
// (they are mutually exclusive by geometry — concyclic holes are not collinear
// and vice-versa, and a circle fit is degenerate on a line):
//   bolt_circle_pattern   — centres on a common circle (radius >= hole dia);
//   linear_hole_array     — centres collinear AND evenly pitched;
//   rectangular_hole_grid — centres on a filled cols×rows lattice (n >= 6).
// Order matters only for the 4-hole degenerate case: 4 grid corners are
// concyclic, so a 2×2 grid is (correctly) reported as a bolt circle; genuine
// grids (>= 2×3) are not concyclic and fall through to the lattice fit.
// Grounded in measured atoms and specific enough not to false-fire on a
// rectangular pocket's concyclic corner fillets (rejected upstream by the
// revolution-completeness gate) — verified on the fpscan panel.
void matchHolePatterns(const std::vector<Hole>& holes,
                       std::vector<skill::RecognizedFeature>& out)
{
    std::vector<bool> used(holes.size(), false);
    for (std::size_t i = 0; i < holes.size(); ++i) {
        if (used[i]) continue;
        // Group holes equal in diameter (±0.1 mm) and parallel in axis.
        std::vector<std::size_t> grp;
        for (std::size_t j = 0; j < holes.size(); ++j) {
            if (used[j]) continue;
            if (std::abs(holes[j].dia - holes[i].dia) > 0.1) continue;
            if (!axisParallel(holes[j].ax, holes[j].ay, holes[j].az,
                              holes[i].ax, holes[i].ay, holes[i].az)) continue;
            grp.push_back(j);
        }
        if (grp.size() < 3) continue;

        std::vector<Hole> pts;
        pts.reserve(grp.size());
        for (std::size_t j : grp) pts.push_back(holes[j]);
        const double dia = holes[i].dia;
        const int    n   = static_cast<int>(grp.size());
        const json   axis = { holes[i].ax, holes[i].ay, holes[i].az };
        // Centres of the consumed holes, so inferProcessPlan can SUBSUME the
        // individual drill steps once the compound replaces them.
        json holeCenters = json::array();
        for (const auto& h : pts) holeCenters.push_back({ h.x, h.y });

        // ── bolt-circle ────────────────────────────────────────────────
        double cx, cy, r, cresid;
        if (fitCircleXY(pts, cx, cy, r, cresid) && cresid <= 0.3 && r >= dia) {
            for (std::size_t j : grp) used[j] = true;
            skill::RecognizedFeature rf;
            rf.skill_id         = "bolt_circle_pattern";
            rf.recovered_params = {
                { "hole_count",         n },
                { "hole_dia_mm",        dia },
                { "bolt_circle_dia_mm", 2.0 * r },
                { "center_x_mm",        cx },
                { "center_y_mm",        cy },
                { "axis_dir",           axis },
                { "hole_centers",       holeCenters },
            };
            rf.confidence       = std::min(0.95, 0.9 + (0.3 - cresid) * 0.1);
            rf.matched_geometry = {
                { "is_compound", true }, { "source", "grammar:bolt_circle" },
                { "fit_residual_mm", cresid }, { "grounded_atom_count", n },
            };
            out.push_back(std::move(rf));
            continue;
        }

        // ── linear hole array ──────────────────────────────────────────
        // (Self-contained: only emit + continue on a match; otherwise fall
        // through to the grid attempt — never `continue` on a non-match here.)
        {
            double dx, dy, lcx, lcy, lresid;
            std::vector<double> proj;
            if (fitLineXY(pts, dx, dy, lcx, lcy, lresid, proj) && lresid <= 0.3) {
                std::sort(proj.begin(), proj.end());
                double meanPitch = 0.0;
                for (std::size_t k = 1; k < proj.size(); ++k)
                    meanPitch += proj[k] - proj[k - 1];
                meanPitch /= static_cast<double>(proj.size() - 1);
                bool ok = meanPitch >= 0.5 * dia;       // not overlapping
                if (ok) {
                    const double pitchTol = std::max(0.1, 0.06 * meanPitch);
                    for (std::size_t k = 1; k < proj.size() && ok; ++k)
                        if (std::abs((proj[k] - proj[k - 1]) - meanPitch) > pitchTol)
                            ok = false;
                }
                if (ok) {
                    for (std::size_t j : grp) used[j] = true;
                    const double t0 = proj.front();
                    skill::RecognizedFeature rf;
                    rf.skill_id         = "linear_hole_array";
                    rf.recovered_params = {
                        { "hole_count",   n },
                        { "hole_dia_mm",  dia },
                        { "pitch_mm",     meanPitch },
                        { "span_mm",      proj.back() - proj.front() },
                        { "start_x_mm",   lcx + dx * t0 },
                        { "start_y_mm",   lcy + dy * t0 },
                        { "direction",    { dx, dy, 0.0 } },
                        { "axis_dir",     axis },
                        { "hole_centers", holeCenters },
                    };
                    rf.confidence       = std::min(0.95, 0.88 + (0.3 - lresid) * 0.1);
                    rf.matched_geometry = {
                        { "is_compound", true }, { "source", "grammar:linear_hole_array" },
                        { "fit_residual_mm", lresid }, { "grounded_atom_count", n },
                    };
                    out.push_back(std::move(rf));
                    continue;
                }
            }
        }

        // ── rectangular hole grid ──────────────────────────────────────
        {
            double pu, pv;  int cols, rows;  double gresid;
            double ox, oy, udx, udy, vdx, vdy;
            if (fitGridXY(pts, pu, pv, cols, rows, gresid,
                          ox, oy, udx, udy, vdx, vdy)) {
                for (std::size_t j : grp) used[j] = true;
                skill::RecognizedFeature rf;
                rf.skill_id         = "rectangular_hole_grid";
                rf.recovered_params = {
                    { "hole_count",  n },
                    { "hole_dia_mm", dia },
                    { "cols",        cols },
                    { "rows",        rows },
                    { "pitch_u_mm",  pu },
                    { "pitch_v_mm",  pv },
                    { "origin_x_mm", ox },
                    { "origin_y_mm", oy },
                    { "u_dir",       { udx, udy, 0.0 } },
                    { "v_dir",       { vdx, vdy, 0.0 } },
                    { "axis_dir",    axis },
                    { "hole_centers", holeCenters },
                };
                rf.confidence       = std::min(0.95, 0.88 + (0.3 - gresid) * 0.1);
                rf.matched_geometry = {
                    { "is_compound", true }, { "source", "grammar:rectangular_hole_grid" },
                    { "fit_residual_mm", gresid }, { "grounded_atom_count", n },
                };
                out.push_back(std::move(rf));
                continue;
            }
        }
    }
}

// PATTERN: coaxial step-bore — >=3 revolution-complete cylinders on a common
// axis line at DISTINCT diameters (a stepped/counterbored bore, the multi-step
// generalisation of bore_with_shelf).  Grounded in the precise bore/drill atoms
// that already recover each cylindrical section; specific because three
// concentric full bores of different size do not occur by accident.
void matchCoaxialStepBores(const std::vector<Hole>& sections,
                           std::vector<skill::RecognizedFeature>& out)
{
    std::vector<bool> used(sections.size(), false);
    for (std::size_t i = 0; i < sections.size(); ++i) {
        if (used[i]) continue;
        std::vector<std::size_t> grp;
        for (std::size_t j = 0; j < sections.size(); ++j) {
            if (used[j]) continue;
            if (!coaxialLines(sections[i].ax, sections[i].ay, sections[i].az,
                              sections[i].x,  sections[i].y,  sections[i].z,
                              sections[j].ax, sections[j].ay, sections[j].az,
                              sections[j].x,  sections[j].y,  sections[j].z)) continue;
            grp.push_back(j);
        }
        // Distinct diameters along this axis (sections of equal dia already
        // merged in assembly; this guards residual near-duplicates).
        std::vector<double> dias;
        for (std::size_t j : grp) {
            bool dup = false;
            for (double d : dias) if (std::abs(d - sections[j].dia) <= 0.1) { dup = true; break; }
            if (!dup) dias.push_back(sections[j].dia);
        }
        if (dias.size() < 3) continue;                  // <3 steps → bore_with_shelf territory

        for (std::size_t j : grp) used[j] = true;
        std::sort(dias.begin(), dias.end(), std::greater<double>());
        json diaArr = json::array();
        for (double d : dias) diaArr.push_back(d);
        skill::RecognizedFeature rf;
        rf.skill_id         = "coaxial_step_bore";
        rf.recovered_params = {
            { "step_count",    static_cast<int>(dias.size()) },
            { "diameters_mm",  diaArr },
            { "max_dia_mm",    dias.front() },
            { "min_dia_mm",    dias.back() },
            { "position_x_mm", sections[i].x },
            { "position_y_mm", sections[i].y },
            { "axis_dir",      { sections[i].ax, sections[i].ay, sections[i].az } },
        };
        rf.confidence       = 0.9;
        rf.matched_geometry = {
            { "is_compound", true }, { "source", "grammar:coaxial_step_bore" },
            { "grounded_atom_count", static_cast<int>(grp.size()) },
        };
        out.push_back(std::move(rf));
    }
}

// Assemble recognizer cylinder atoms into physical sections: merge atoms that
// share a location (within tol), a parallel axis and a diameter; sum each
// distinct face's wrap so a section split across STEP faces still reads as a
// full revolution.  `includeBores` adds bore_cylindrical atoms (wide bores that
// drill_hole may not own), used by the coaxial step-bore grammar.
std::vector<Hole> assembleHoles(const std::vector<skill::RecognizedFeature>& candidates,
                                bool includeBores = false)
{
    std::vector<Hole> holes;
    for (const auto& c : candidates) {
        const bool isDrill = c.skill_id == "drill_hole" ||
                             c.skill_id == "drill_through_hole";
        const bool isBore  = includeBores && c.skill_id == "bore_cylindrical";
        if (!isDrill && !isBore) continue;
        if (c.confidence < 0.7) continue;
        const json& p = c.recovered_params;
        const double dia = num(p, "diameter_mm");
        if (dia <= 0.0) continue;

        Hole a;
        a.x = num(p, "position_x_mm");
        a.y = num(p, "position_y_mm");
        a.z = num(p, "position_z_mm");
        a.dia = dia;
        a.ax = 0.0; a.ay = 0.0; a.az = -1.0;
        if (p.contains("axis_dir") && p["axis_dir"].is_array() &&
            p["axis_dir"].size() == 3) {
            a.ax = p["axis_dir"][0].get<double>();
            a.ay = p["axis_dir"][1].get<double>();
            a.az = p["axis_dir"][2].get<double>();
        }
        const json& mg = c.matched_geometry;
        const double wrap = mg.is_object() ? mg.value("cyl_wrap_rad", 0.0) : 0.0;
        const int faceId  = (mg.is_object() && mg.contains("cylindrical_face_id") &&
                             mg["cylindrical_face_id"].is_number_integer())
                                ? mg["cylindrical_face_id"].get<int>() : -1;

        // Merge into an existing hole sharing XY (within tol), axis and dia.
        Hole* host = nullptr;
        for (auto& h : holes) {
            if (std::abs(h.dia - a.dia) > 0.1) continue;
            if (!axisParallel(h.ax, h.ay, h.az, a.ax, a.ay, a.az)) continue;
            if (std::hypot(h.x - a.x, h.y - a.y) > kColocTolMm) continue;
            host = &h; break;
        }
        if (!host) { holes.push_back(a); host = &holes.back(); }
        // Count each distinct face's wrap once (drill_hole + drill_through_hole
        // report the SAME face; only the first should contribute its arc).
        if (faceId < 0 || host->faces.insert(faceId).second)
            host->wrap += wrap;
    }
    return holes;
}

}  // namespace

std::vector<skill::RecognizedFeature>
recognizeCompounds(const std::vector<skill::RecognizedFeature>& candidates)
{
    // Assemble precise drill atoms into physical holes, then keep only the ones
    // whose faces complete a revolution — real holes, not concave corner
    // fillets — as the grounding evidence for hole-pattern grammars.
    std::vector<Hole> holes;
    for (auto& h : assembleHoles(candidates))
        if (h.wrap >= kHoleWrapMin) holes.push_back(std::move(h));

    std::vector<skill::RecognizedFeature> out;
    matchHolePatterns(holes, out);

    // Coaxial step-bore works over the same revolution-complete sections, but
    // INCLUDING wide bore_cylindrical atoms and keeping distinct diameters on a
    // shared axis line (orthogonal to the equal-diameter hole patterns above).
    std::vector<Hole> sections;
    for (auto& h : assembleHoles(candidates, /*includeBores=*/true))
        if (h.wrap >= kHoleWrapMin) sections.push_back(std::move(h));
    matchCoaxialStepBores(sections, out);
    return out;
}

}  // namespace koocadcam::re
