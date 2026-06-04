// @lat: [[engine/skills#heatsink_fin_pin_array_compound]]

#include "heatsink_fin_pin_array_compound.hpp"

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
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::heatsink_fin_pin_array_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pattern != "grid" && in.pattern != "hex") {
        r.add("DFM-PIN-LAYOUT", "error",
              "heatsink_fin_pin_array_compound: pattern must be 'grid' or 'hex' (got '" +
              in.pattern + "')");
    }
    if (!(in.pin_dia_mm > 0.5)) {
        r.add("DFM-PIN-DIA", "error",
              "heatsink_fin_pin_array_compound: pin_dia_mm " +
              std::to_string(in.pin_dia_mm) +
              " <= 0.5 mm (DFM-002 derived: micro-drill flute breakage)");
    }
    if (!(in.pin_height_mm > 0.0)) {
        r.add("DFM-PIN-HEIGHT", "error",
              "heatsink_fin_pin_array_compound: pin_height_mm must be > 0");
    }
    if (in.fin_count_x < 2 || in.fin_count_y < 2) {
        r.add("DFM-PIN-N", "error",
              "heatsink_fin_pin_array_compound: fin_count_x and fin_count_y must be >= 2");
    }
    if (in.pin_dia_mm > 0.5 && in.pin_height_mm > 0.0) {
        const double ar = in.pin_height_mm / in.pin_dia_mm;
        if (ar > 15.0) {
            r.add("DFM-PIN-AR", "error",
                  "heatsink_fin_pin_array_compound: pin aspect ratio " +
                  std::to_string(ar) + " > 15 (un-machinable / un-castable)");
        }
    }
    if (in.fin_pitch_x_mm > 0.0 && in.pin_dia_mm > 0.5 &&
        in.fin_pitch_x_mm < in.pin_dia_mm + 0.5) {
        r.add("DFM-PIN-PITCH", "error",
              "heatsink_fin_pin_array_compound: fin_pitch_x " +
              std::to_string(in.fin_pitch_x_mm) +
              " < pin_dia + 0.5 (no fin gap)");
    }
    if (in.fin_pitch_y_mm > 0.0 && in.pin_dia_mm > 0.5 &&
        in.fin_pitch_y_mm < in.pin_dia_mm + 0.5) {
        r.add("DFM-PIN-PITCH", "error",
              "heatsink_fin_pin_array_compound: fin_pitch_y " +
              std::to_string(in.fin_pitch_y_mm) +
              " < pin_dia + 0.5 (no fin gap)");
    }
    // Fit check
    if (in.fin_count_x >= 2 && in.fin_count_y >= 2 &&
        in.fin_pitch_x_mm > 0.0 && in.fin_pitch_y_mm > 0.0) {
        const double spanX = (in.fin_count_x - 1) * in.fin_pitch_x_mm + in.pin_dia_mm;
        const double spanY = (in.fin_count_y - 1) * in.fin_pitch_y_mm + in.pin_dia_mm;
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        if (spanX > (xMax - xMin) + 1e-3 || spanY > (yMax - yMin) + 1e-3) {
            r.add("DFM-PIN-FIT", "error",
                  "heatsink_fin_pin_array_compound: array span (" +
                  std::to_string(spanX) + " x " + std::to_string(spanY) +
                  ") mm exceeds base face bbox");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "heatsink_fin_pin_array_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;

    constexpr double kEmbed = 0.05;
    const double pinR = in.pin_dia_mm * 0.5;

    // Generate centres for grid or hex pattern.
    std::vector<std::pair<double,double>> centres;
    centres.reserve(static_cast<std::size_t>(in.fin_count_x * in.fin_count_y));
    if (in.pattern == "grid") {
        for (int j = 0; j < in.fin_count_y; ++j) {
            for (int i = 0; i < in.fin_count_x; ++i) {
                centres.emplace_back(
                    in.origin_x_mm + i * in.fin_pitch_x_mm,
                    in.origin_y_mm + j * in.fin_pitch_y_mm);
            }
        }
    } else {  // "hex" — odd rows shifted by px/2 (slice-12 fill_pattern style)
        for (int j = 0; j < in.fin_count_y; ++j) {
            const double xShift = (j % 2 == 0) ? 0.0 : in.fin_pitch_x_mm * 0.5;
            for (int i = 0; i < in.fin_count_x; ++i) {
                centres.emplace_back(
                    in.origin_x_mm + xShift + i * in.fin_pitch_x_mm,
                    in.origin_y_mm + j * in.fin_pitch_y_mm);
            }
        }
    }

    // Build pin cylinders and fuse them onto the workpiece.
    std::vector<TopoDS_Shape> pins;
    pins.reserve(centres.size());
    for (const auto& c : centres) {
        const gp_Pnt origin(c.first, c.second, zTop - kEmbed);
        const gp_Ax2 ax(origin, gp::DZ());
        pins.push_back(pr::cylinder(ax, pinR, in.pin_height_mm + kEmbed));
    }
    if (pins.empty())
        throw SkillError("heatsink_fin_pin_array_compound: no pins generated");

    const TopoDS_Shape newShape = pr::fuseMany(wp.shape(), pins);

    const int totalPins   = static_cast<int>(centres.size());
    const double pinVol   = M_PI * pinR * pinR * in.pin_height_mm;
    const double totalVol = pinVol * totalPins;

    json params = {
        { "face_id",         in.face_id },
        { "fin_pitch_x_mm",  in.fin_pitch_x_mm },
        { "fin_pitch_y_mm",  in.fin_pitch_y_mm },
        { "fin_count_x",     in.fin_count_x },
        { "fin_count_y",     in.fin_count_y },
        { "pin_dia_mm",      in.pin_dia_mm },
        { "pin_height_mm",   in.pin_height_mm },
        { "pattern",         in.pattern },
        { "origin_x_mm",     in.origin_x_mm },
        { "origin_y_mm",     in.origin_y_mm },
    };
    json pattern_js = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "electronics_feature_type", "pin_fin_heat_sink" },
        { "pattern_layout",         in.pattern },
        { "subfeature_count",       totalPins },
        { "pin_count_x",            in.fin_count_x },
        { "pin_count_y",            in.fin_count_y },
        { "pin_dia_mm",             in.pin_dia_mm },
        { "pin_height_mm",          in.pin_height_mm },
        { "fin_pitch_x_mm",         in.fin_pitch_x_mm },
        { "fin_pitch_y_mm",         in.fin_pitch_y_mm },
        { "pin_aspect_ratio",       in.pin_height_mm / std::max(1e-9, in.pin_dia_mm) },
        { "derived_volume_added_mm3", totalVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "extrusion;mill;cast";
    tooling.tool_dia_mm       = in.pin_dia_mm;
    tooling.tool_length_mm    = in.pin_height_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = -totalVol;     // additive
    tooling.est_cycle_time_s  = std::max(5.0, totalPins * 1.5);
    tooling.extra = {
        { "operation_note",
          "Cast/forge pin-fin heat sink, or mill grid from solid plate." },
        { "pin_count",      totalPins },
        { "added_volume_mm3", totalVol },
    };

    FeatureSignature sig { kSkillId, params, pattern_js, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::heatsink_fin_pin_array_compound applied: {}x{}={} pins ({} mm dia, {} mm tall, {} layout)",
        in.fin_count_x, in.fin_count_y, totalPins,
        in.pin_dia_mm, in.pin_height_mm, in.pattern);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy:
//   1. Metadata replay (confidence 0.95).
//   2. Geometric fallback: count cylindrical bosses of the same radius —
//      this matches the slice-12 fill_pattern recognizer.  If >= 4 cylinders
//      of equal radius are found, emit a low-confidence guess.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.95;
        r.matched_geometry = {
            { "source",      "metadata_replay" },
            { "is_compound", true },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric back-map: count cylindrical faces of equal radius.
    std::vector<double> radii;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder gc = s.Cylinder();
            radii.push_back(gc.Radius());
        } catch (...) {}
    }
    if (radii.size() < 4) return out;

    std::sort(radii.begin(), radii.end());
    int bestCount = 1, run = 1;
    double bestR = radii.front();
    for (std::size_t i = 1; i < radii.size(); ++i) {
        if (std::abs(radii[i] - radii[i - 1]) < 1e-2) {
            ++run;
            if (run > bestCount) { bestCount = run; bestR = radii[i]; }
        } else {
            run = 1;
        }
    }
    if (bestCount < 4) return out;

    json recovered = {
        { "face_id",     -1 },
        { "pin_dia_mm",  2.0 * bestR },
        { "pattern",     "grid" },
        { "pin_count_total", bestCount },
    };
    json matched = {
        { "source",         "geometric_pin_bosses" },
        { "cyl_count",      bestCount },
        { "dominant_radius_mm", bestR },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    return out;
}

}  // namespace koocadcam::skill::heatsink_fin_pin_array_compound
