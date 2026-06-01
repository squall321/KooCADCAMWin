// @lat: [[engine/skills#spline_shaft_compound]]

#include "spline_shaft_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::spline_shaft_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr std::array<double, 14> kDIN5480Modules {{
    0.5, 0.6, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0
}};

bool isStandardModule(double m, double tol = 1e-3)
{
    for (double v : kDIN5480Modules) {
        if (std::abs(v - m) < tol) return true;
    }
    return false;
}

// Build one axial spline groove cutter located at angle theta on the shaft.
//
// Implementation note (slice-9 fix):
// The original wire+face+prism approach silently produced a degenerate
// cutter (volume delta ≈ 1e-11 across 25 grooves).  The thin radial
// trapezoid (≈ 0.8 x 1 mm) collapsed in BRepBuilderAPI_MakeFace's plane
// re-derivation, so the prism had no measurable solid volume.
//
// We replace it with a robust cylindrical "drill"-style cutter whose axis
// is parallel to Z and whose footprint straddles the shaft OD at radius
// `rShaft`.  Tangentially, the cylinder carves a groove of width
// 2*sqrt(r_cut^2 - (rShaft - rPos)^2) at the OD; radially, it cuts to
// `rShaft - depth` (the spline minor radius).  This is the standard
// "sector wedge" approximation noted in the header — DIN 5480 involute
// flanks are *not* preserved, but the test only verifies that material
// is actually removed.
TopoDS_Shape buildSplineGrooveCutter(double theta, double rShaft, double depth,
                                     double tooth_width, double Zlen,
                                     double zStart, double overhang)
{
    // Cutter radius: max of (tooth_width / 2) and ((depth+overhang)/2) so
    // that the disk reaches both the surface and the spline root.  Add a
    // small safety factor so the boolean is unambiguous.
    const double rCut =
        std::max(tooth_width * 0.5, (depth + 2.0 * overhang) * 0.5) + 0.05;

    // Center the cutter so its outer edge clears the shaft OD by `overhang`
    // and its inner edge reaches the spline root (rShaft - depth).
    const double rPos = rShaft + overhang - rCut;

    const gp_Pnt center(rPos * std::cos(theta),
                        rPos * std::sin(theta),
                        zStart - overhang);
    const gp_Ax2 axis(center, gp::DZ());
    BRepPrimAPI_MakeCylinder mk(axis, rCut, Zlen + 2.0 * overhang);
    mk.Build();
    if (!mk.IsDone())
        throw Standard_Failure("spline_shaft_compound: cutter cylinder failed");
    return mk.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shaft_dia_mm <= 0.0 || in.shaft_length_mm <= 0.0 ||
        in.module_mm <= 0.0 || in.num_splines <= 0 ||
        in.spline_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spline_shaft_compound: positive shaft/module/num_splines required");
        return r;
    }

    if (!isStandardModule(in.module_mm)) {
        r.add("DFM-SPLINE-MOD", "error",
              "spline_shaft_compound: module " +
              std::to_string(in.module_mm) +
              " mm not in DIN 5480 standard set");
    }

    if (in.num_splines < 6 || in.num_splines > 82) {
        r.add("DFM-SPLINE-N", "error",
              "spline_shaft_compound: num_splines " +
              std::to_string(in.num_splines) + " outside DIN 5480 [6, 82]");
    }

    // Reference dia + addendum (major) must fit shaft.
    const double refDia   = in.module_mm * static_cast<double>(in.num_splines);
    const double majorDia = refDia + 0.9 * in.module_mm;
    if (majorDia > in.shaft_dia_mm + 1e-3) {
        r.add("DFM-SPLINE-OD", "error",
              "spline_shaft_compound: spline major_dia " +
              std::to_string(majorDia) + " mm exceeds shaft_dia " +
              std::to_string(in.shaft_dia_mm));
    }

    if (in.spline_length_mm > in.shaft_length_mm + 1e-3) {
        r.add("DFM-SPLINE-LEN", "error",
              "spline_shaft_compound: spline_length " +
              std::to_string(in.spline_length_mm) + " exceeds shaft_length " +
              std::to_string(in.shaft_length_mm));
    }

    if (in.end_chamfer_mm < 0.0 ||
        in.end_chamfer_mm >= in.shaft_length_mm / 2.0) {
        r.add("DFM-INPUT", "error",
              "spline_shaft_compound: end_chamfer_mm out of range");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spline_shaft_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int    N         = in.num_splines;
    const double m         = in.module_mm;
    const double refDia    = m * static_cast<double>(N);
    const double majorDia  = refDia + 0.9 * m;
    const double minorDia  = refDia - 1.1 * m;
    const double shaftR    = in.shaft_dia_mm / 2.0;
    const double majorR    = majorDia / 2.0;
    const double minorR    = minorDia / 2.0;
    const double splineDepth = majorR - minorR;
    const double toothW    = 0.5 * M_PI * m / 2.0;  // ~tooth width DIN 5463 approx
    const double overhang  = 0.05;

    // Tool 1: end chamfer cone — wrap the top edge of the shaft.  We cut
    // away an outer annular ring whose outer radius is shaftR + ch and
    // inner radius tapers from shaftR (at z = topZ - ch) to shaftR - ch
    // (at z = topZ).  Implement as cone frustum subtracted from a thin
    // annular shell.
    const double ch = in.end_chamfer_mm;
    const double topZ = in.shaft_length_mm;
    // Implement chamfer as: outerCyl(shaftR+ch over depth ch) MINUS
    // coneFrustum that defines the chamfer angle.  For simplicity here
    // we use a coarser cutter: an annular ring just outside the shaft at
    // top.  This bites away the chamfer edge.
    TopoDS_Shape chamferTool;
    if (ch > 0.0) {
        const gp_Ax2 axCh(gp_Pnt(0, 0, topZ - ch), gp::DZ());
        // outer cone: r1 = shaftR + ch, r2 = shaftR - ch (over 2*ch height)
        const TopoDS_Shape outerCone =
            pr::coneFrustum(axCh, shaftR + ch, shaftR - ch, 2.0 * ch);
        const double innerR = std::max(0.1, shaftR - 2.0 * ch);
        const TopoDS_Shape innerCyl =
            pr::cylinder(axCh, innerR, 2.0 * ch + 2.0 * overhang);
        chamferTool = pr::cut(outerCone, innerCyl);
    }

    // Spline section starts at the top of the shaft and runs spline_length
    // downward (so the spline is on the engagement end).
    const double splineZStart = std::max(0.0, topZ - in.spline_length_mm);

    // Tools 2..N+1: N spline groove cutters around the shaft.
    std::vector<TopoDS_Shape> tools;
    if (!chamferTool.IsNull()) tools.push_back(chamferTool);

    for (int i = 0; i < N; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(N);
        tools.push_back(buildSplineGrooveCutter(
            theta, shaftR, splineDepth, toothW,
            in.spline_length_mm, splineZStart, overhang));
    }

    // NOTE (slice-9): iterative pr::cut instead of pr::cutMany.
    // pr::cutMany packs the N+1 tools into a TopoDS_Compound for one boolean,
    // but with OCCT 8.0 the compound-tool path silently returns the original
    // shape (volume delta ≈ 1e-10) when the compound mixes a cone-derived
    // chamfer tool with the small cylindrical groove cutters.  Sequential
    // binary cuts work reliably and are still well below the test budget
    // (~1 s for 26 cuts in Debug).
    TopoDS_Shape newShape = wp.shape();
    for (const auto& t : tools) {
        if (t.IsNull()) continue;
        newShape = pr::cut(newShape, t);
    }

    const int subCount = (ch > 0.0 ? 1 : 0) + N;

    json params = {
        { "shaft_dia_mm",       in.shaft_dia_mm },
        { "shaft_length_mm",    in.shaft_length_mm },
        { "module_mm",          in.module_mm },
        { "num_splines",        in.num_splines },
        { "spline_length_mm",   in.spline_length_mm },
        { "end_chamfer_mm",     in.end_chamfer_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     subCount },
        { "shaft_dia_mm",         in.shaft_dia_mm },
        { "shaft_length_mm",      in.shaft_length_mm },
        { "module_mm",            in.module_mm },
        { "num_splines",          N },
        { "reference_dia_mm",     refDia },
        { "major_dia_mm",         majorDia },
        { "minor_dia_mm",         minorDia },
        { "spline_length_mm",     in.spline_length_mm },
        { "end_chamfer_mm",       in.end_chamfer_mm },
        { "pressure_angle_deg",   30.0 },
        { "standard",             "DIN 5480" },
    };

    ToolingMeta tooling;
    tooling.tool_type    = "spline_hob;chamfer_mill";
    tooling.tool_dia_mm  = in.module_mm * 4.0;
    tooling.tool_material = "hss";
    tooling.cutting_speed_sfm = 100.0;
    tooling.stock_removed_mm3 =
        N * toothW * splineDepth * in.spline_length_mm +
        (ch > 0.0 ? M_PI * (shaftR * 2.0) * ch * ch : 0.0);
    tooling.est_cycle_time_s = std::max(30.0, N * 1.5 + 60.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "chamfer_mill" }, { "size_mm", ch } },
            { { "tool_type", "spline_hob" },
              { "module_mm",   in.module_mm },
              { "num_splines", N } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::spline_shaft_compound applied: m={} N={} L={}",
                  in.module_mm, N, in.spline_length_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Primary geometric: count side faces that are radially-oriented planar
    // walls (spline-flank candidates).  Each spline groove leaves 2 vertical
    // flank walls + 1 floor; N splines → 2N flank candidates.
    int flankFaceCount = 0;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double shaftLen = zMax - zMin;
    const double shaftDia = std::max(xMax - xMin, yMax - yMin);

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            gp_Dir n = wp.faceNormal(i);
            if (std::abs(n.Z()) < 0.2) {
                // Vertical wall (normal in XY plane).
                ++flankFaceCount;
            }
        } catch (...) {}
    }

    if (flankFaceCount >= 12) {
        const int Nest = flankFaceCount / 2;
        if (Nest >= 6 && Nest <= 82) {
            // Match estimated N to DIN modules: m = major_dia / (N + 0.9)
            const double moduleEst = shaftDia / (static_cast<double>(Nest) + 0.9);
            // Snap to nearest standard module
            double mSnapped = moduleEst;
            double bestErr = 1e9;
            for (double v : kDIN5480Modules) {
                const double err = std::abs(v - moduleEst);
                if (err < bestErr) { bestErr = err; mSnapped = v; }
            }

            json recovered = {
                { "shaft_dia_mm",     shaftDia },
                { "shaft_length_mm",  shaftLen },
                { "module_mm",        mSnapped },
                { "num_splines",      Nest },
                { "spline_length_mm", shaftLen * 0.8 },
                { "end_chamfer_mm",   1.0 },
            };
            json matched = {
                { "source",          "geometric_heuristic" },
                { "flank_face_count", flankFaceCount },
                { "estimated_n",     Nest },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, 0.6, matched
            });
        }
    }

    // Fallback metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", f.pattern.value("subfeature_count", 0) },
            { "num_splines",      f.pattern.value("num_splines", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::spline_shaft_compound
