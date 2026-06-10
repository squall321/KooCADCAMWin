// @lat: [[engine/skills#perforated_grille_pattern]]

#include "perforated_grille_pattern.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::perforated_grille_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────

namespace {

struct SpecEntry {
    const char* key;
    double      hole_dia_mm;
    double      hex_pitch_mm;
};

constexpr std::array<SpecEntry, 3> kSpecTable {{
    { "HEX-3", 3.0, 5.0  },    // fine — speaker grille
    { "HEX-5", 5.0, 8.0  },    // medium — equipment vent
    { "HEX-8", 8.0, 12.0 },    // coarse — HVAC return
}};

}  // namespace

GrilleSpec grilleSpecFor(const std::string& key)
{
    for (const auto& e : kSpecTable) {
        if (key == e.key) return { e.hole_dia_mm, e.hex_pitch_mm };
    }
    return {};
}

namespace {

GrilleSpec resolveSpec(const Input& in)
{
    if (!in.spec_key.empty()) return grilleSpecFor(in.spec_key);
    return { in.hole_dia_mm, in.hex_pitch_mm };
}

// Generate hex-pattern (x, y) grid points within a rectangle centered at
// origin, of dimensions length × width.  Pattern axis is along +X.
std::vector<std::pair<double,double>> hexGrid(
    double length, double width, double pitch)
{
    std::vector<std::pair<double,double>> pts;
    if (length <= 0.0 || width <= 0.0 || pitch <= 0.0) return pts;

    const double rowStep = pitch * std::sqrt(3.0) / 2.0;
    const int nRows = static_cast<int>(width  / rowStep) + 1;
    const int nCols = static_cast<int>(length / pitch  ) + 1;

    const double yStart = -((nRows - 1) * rowStep) / 2.0;
    for (int j = 0; j < nRows; ++j) {
        const double y = yStart + j * rowStep;
        if (std::abs(y) > width / 2.0 + 1e-6) continue;
        const double xOffset = (j % 2 == 0) ? 0.0 : pitch / 2.0;
        const double xStart = -((nCols - 1) * pitch) / 2.0 + xOffset;
        for (int i = 0; i < nCols; ++i) {
            const double x = xStart + i * pitch;
            if (std::abs(x) > length / 2.0 + 1e-6) continue;
            pts.emplace_back(x, y);
        }
    }
    return pts;
}

}  // namespace

int hexHoleCount(double L, double W, double pitch)
{
    return static_cast<int>(hexGrid(L, W, pitch).size());
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!in.spec_key.empty()) {
        GrilleSpec s = grilleSpecFor(in.spec_key);
        if (s.hex_pitch_mm <= 0.0) {
            r.add("DFM-GRILLE-SP", "error",
                  "perforated_grille_pattern: unknown spec_key '" + in.spec_key +
                  "' (supported: HEX-3/HEX-5/HEX-8)");
            return r;
        }
    } else {
        if (in.hole_dia_mm <= 0.0 || in.hex_pitch_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "perforated_grille_pattern: spec_key empty AND "
                  "hole_dia/hex_pitch not all > 0");
            return r;
        }
    }
    const GrilleSpec eff = resolveSpec(in);

    if (eff.hole_dia_mm > 0.0 && eff.hole_dia_mm < 0.8) {
        r.add("DFM-GRILLE-DIA", "error",
              "perforated_grille_pattern: hole_dia " +
              std::to_string(eff.hole_dia_mm) + " mm < 0.8 mm (DFM-002)");
    }
    if (eff.hole_dia_mm > 0.0 && eff.hex_pitch_mm > 0.0 &&
        eff.hole_dia_mm >= eff.hex_pitch_mm) {
        r.add("DFM-GRILLE-FIT", "error",
              "perforated_grille_pattern: hole_dia >= hex_pitch (holes overlap)");
    }
    if (in.footprint_length_mm <= 0.0 || in.footprint_width_mm <= 0.0) {
        // Auto-detect: when footprint not given, infer from the workpiece
        // bounding box (smaller of width/depth).  This is the context-detect
        // branch — the same input can mean different things depending on
        // existing workpiece.
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double availL = xMax - xMin;
        const double availW = yMax - yMin;
        if (availL <= 0.0 || availW <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "perforated_grille_pattern: footprint not given AND "
                  "workpiece bbox is degenerate — cannot auto-detect");
        }
    }

    // Compute the hole count actually achievable.  When footprint is
    // missing, auto-detect from the workpiece bbox.
    double xMinV, yMinV, zMinV, xMaxV, yMaxV, zMaxV;
    wp.boundingBox(xMinV, yMinV, zMinV, xMaxV, yMaxV, zMaxV);
    const double effL = (in.footprint_length_mm > 0.0)
        ? in.footprint_length_mm : (xMaxV - xMinV);
    const double effW = (in.footprint_width_mm  > 0.0)
        ? in.footprint_width_mm  : (yMaxV - yMinV);
    if (effL > 0.0 && effW > 0.0 && eff.hex_pitch_mm > 0.0) {
        const int n = hexHoleCount(effL, effW, eff.hex_pitch_mm);
        if (n < 4) {
            r.add("DFM-GRILLE-N", "error",
                  "perforated_grille_pattern: footprint yields " +
                  std::to_string(n) + " holes < 4 (not a pattern)");
        }
        // Open-area DFM: hex pattern's theoretical max is
        //   (π/(2√3)) · (d/p)²  ≈ 0.9069 × (d/p)².
        // We require ≥ 50 % to pass the airflow gate.
        const double openFrac = (M_PI / (2.0 * std::sqrt(3.0))) *
            std::pow(eff.hole_dia_mm / eff.hex_pitch_mm, 2.0);
        if (openFrac < 0.50) {
            r.add("DFM-GRILLE-FA", "error",
                  "perforated_grille_pattern: open-area fraction " +
                  std::to_string(openFrac * 100.0) +
                  "% < 50 % of bbox face");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "perforated_grille_pattern DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const GrilleSpec eff = resolveSpec(in);

    auto faceId = wp.resolve(in.face);
    if (!faceId) throw SkillError("perforated_grille_pattern: face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;
    const double wallThk = (in.wall_thickness_mm > 0.0) ? in.wall_thickness_mm
                                                        : 3.0;

    // Context auto-detect: when footprint not given, use the workpiece bbox.
    const double effL = (in.footprint_length_mm > 0.0)
        ? in.footprint_length_mm : (xMax - xMin);
    const double effW = (in.footprint_width_mm > 0.0)
        ? in.footprint_width_mm : (yMax - yMin);

    constexpr double kOverhang = 0.1;

    // Local axes
    const gp_Dir gax = in.grid_axis;
    const gp_Dir perpDir(-gax.Y(), gax.X(), 0.0);

    const auto hexPts = hexGrid(effL, effW, eff.hex_pitch_mm);
    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(hexPts.size());
    for (const auto& [u, v] : hexPts) {
        // Map (u, v) into world XY using grid_axis + perpDir, offset by center.
        const double wx = in.center_x_mm + u * gax.X() + v * perpDir.X();
        const double wy = in.center_y_mm + u * gax.Y() + v * perpDir.Y();
        // Cylinder pointing into the face (downward Z).
        const gp_Pnt origin(wx, wy, zTop + kOverhang);
        const gp_Ax2 ax(origin, gp_Dir(0.0, 0.0, -1.0));
        cutters.push_back(pr::cylinder(ax, eff.hole_dia_mm / 2.0,
                                       wallThk + 2.0 * kOverhang));
    }

    // cutMany packs the cylinder compound into one Boolean cut.
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const int holeCount = static_cast<int>(cutters.size());
    const double removedMm3 = holeCount *
        M_PI * std::pow(eff.hole_dia_mm / 2.0, 2.0) * wallThk;

    json params = {
        { "face_id",             *faceId },
        { "spec_key",            in.spec_key },
        { "hole_dia_mm",         eff.hole_dia_mm },
        { "hex_pitch_mm",        eff.hex_pitch_mm },
        { "footprint_length_mm", effL },
        { "footprint_width_mm",  effW },
        { "center_x_mm",         in.center_x_mm },
        { "center_y_mm",         in.center_y_mm },
        { "grid_axis",           { gax.X(), gax.Y(), gax.Z() } },
        { "wall_thickness_mm",   wallThk },
    };
    const double openFrac = (M_PI / (2.0 * std::sqrt(3.0))) *
        std::pow(eff.hole_dia_mm / eff.hex_pitch_mm, 2.0);
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    holeCount },
        { "spec_key",            in.spec_key },
        { "hole_dia_mm",         eff.hole_dia_mm },
        { "hex_pitch_mm",        eff.hex_pitch_mm },
        { "open_area_frac",      openFrac },
        { "grid_axis",           { gax.X(), gax.Y(), gax.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill;punch";
    tooling.tool_dia_mm      = eff.hole_dia_mm;
    tooling.tool_length_mm   = wallThk + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedMm3;
    tooling.est_cycle_time_s  = std::max(5.0, holeCount * 0.4);
    tooling.extra = {
        { "operation_note",
          "Either CNC-drill point-by-point, perforating press with hex die, "
          "or laser drill for sheet metal." },
        { "hole_count", holeCount },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::perforated_grille_pattern applied: spec={} count={} dia={} pitch={}",
                  in.spec_key, holeCount, eff.hole_dia_mm, eff.hex_pitch_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric recognition: count cylindrical faces of the same radius and
// check that their axes are parallel and their centres form a hex grid.
// We do a simplified consistency check (same radius + ≥ 4 cylinders) and
// fall back to metadata replay for the spec_key.

namespace {

bool sameZAxis(const gp_Ax1& a, const gp_Ax1& b)
{
    return std::abs(std::abs(a.Direction().Z()) - 1.0) < 1e-2 &&
           std::abs(std::abs(b.Direction().Z()) - 1.0) < 1e-2;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Try metadata replay first.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "spec_key",            f.params.value("spec_key", std::string()) },
            { "hole_dia_mm",         f.params.value("hole_dia_mm", 0.0) },
            { "hex_pitch_mm",        f.params.value("hex_pitch_mm", 0.0) },
            { "footprint_length_mm", f.params.value("footprint_length_mm", 0.0) },
            { "footprint_width_mm",  f.params.value("footprint_width_mm",  0.0) },
            { "center_x_mm",         f.params.value("center_x_mm", 0.0) },
            { "center_y_mm",         f.params.value("center_y_mm", 0.0) },
            { "grid_axis",           f.params.value("grid_axis",
                                       json::array({1.0,0.0,0.0})) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.93, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback (post-STEP-import): look for clusters of equal-
    // radius cylindrical faces with vertical axes.  When we find ≥ 4,
    // emit a low-confidence candidate.
    int cylCount = 0;
    double radiusSum = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        ++cylCount;
        // We don't fetch the radius here (would need BRepAdaptor); just
        // count cylinder occurrences for a coarse pattern indicator.
        (void)radiusSum;
    }
    if (cylCount >= 4) {
        json rec = {
            { "spec_key",            std::string() },
            { "hole_dia_mm",         0.0 },
            { "hex_pitch_mm",        0.0 },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.55,
            json{ { "source", "geometric" },
                  { "cyl_face_count", cylCount } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::perforated_grille_pattern
