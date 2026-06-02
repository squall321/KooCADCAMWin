// @lat: [[engine/skills#form_tap]]
//
// form_tap — cold-forming (chipless) tap.  Same V-thread helical geometry
// as rigid_tap, distinguished by the form-tap-specific pilot diameter
// formula, the ductile-material restriction, and tooling metadata.

#include "form_tap.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/HelicalSweep.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace koocadcam::skill::form_tap {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

// ── Chart lookups ────────────────────────────────────────────────────────
//
// standardCoarsePitch uses central thread_table for M3..M12; M1.6/M2/M2.5
// are below the central table's M3 floor and stay inline.
// formTapPilotDiameter is form-tap-SPECIFIC (chipless displacement) and
// remains entirely local — central table holds cutting-tap pilots.

double standardCoarsePitch(const std::string& thread_size)
{
    if (thread_size == "M1.6") return 0.35;
    if (thread_size == "M2")   return 0.40;
    if (thread_size == "M2.5") return 0.45;
    if (const auto* s = tt::findMetric(thread_size)) return s->pitch_mm;
    return 0.0;
}

double formTapPilotDiameter(const std::string& thread_size)
{
    // form-tap pilot ≈ nominal_dia − 0.5 × pitch  (chart values pre-computed
    // for the standard coarse series).  Larger than the cutting-tap pilot
    // because the form tap displaces, rather than removes, material.
    if (thread_size == "M1.6") return 1.45;     // 1.6 − 0.5×0.35 = 1.425, ≈ 1.45 mm drill
    if (thread_size == "M2")   return 1.80;     // 2.0 − 0.5×0.40 = 1.80
    if (thread_size == "M2.5") return 2.30;     // 2.5 − 0.5×0.45 = 2.275
    if (thread_size == "M3")   return 2.75;     // 3.0 − 0.5×0.50 = 2.75
    if (thread_size == "M4")   return 3.65;     // 4.0 − 0.5×0.70 = 3.65
    if (thread_size == "M5")   return 4.60;     // 5.0 − 0.5×0.80 = 4.60
    if (thread_size == "M6")   return 5.50;     // 6.0 − 0.5×1.00 = 5.50
    if (thread_size == "M8")   return 7.40;     // 8.0 − 0.5×1.25 = 7.375
    if (thread_size == "M10")  return 9.25;     // 10  − 0.5×1.50 = 9.25
    return 0.0;
}

bool isDuctileMaterial(const std::string& material)
{
    // The list mirrors the form-tap manufacturer recommendations
    // (Emuge-Franken, OSG, Yamawa).  Note: 304 / 316 stainless are
    // typically marked "marginal" — we conservatively include them.
    static const std::unordered_set<std::string> ductile = {
        "aluminum_6061", "aluminum_7075", "aluminum",
        "copper", "brass", "bronze",
        "mild_steel", "low_carbon_steel", "1018", "12L14",
        "stainless_304", "stainless_316",
    };
    return ductile.count(material) > 0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "form_tap pitch_mm must be > 0");
    }
    if (in.thread_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "form_tap thread_depth_mm must be > 0");
    }
    if (in.pitch_mm > 0.0 && in.thread_depth_mm < in.pitch_mm * 1.5) {
        r.add("DFM-TAP-ENGAGE", "error",
              "thread_depth_mm " + std::to_string(in.thread_depth_mm) +
              " < 1.5 × pitch (" + std::to_string(in.pitch_mm * 1.5) +
              ") — insufficient thread engagement");
    }

    // DFM-FORM-TAP-MAT — material must be in the ductile list.  This is
    // INFO-level only (the caller can override at their own risk; the
    // finding is for downstream CAPP / cost reporting).
    if (!isDuctileMaterial(wp.material())) {
        r.add("DFM-FORM-TAP-MAT", "info",
              "form_tap requires ductile material; workpiece material '" +
              wp.material() +
              "' is not on the approved ductile list — consider rigid_tap instead");
    }

    // DFM-FORM-TAP-DIA — pilot diameter must match the FORM-TAP chart
    // (DIFFERENT formula from cutting taps).
    const double expected = formTapPilotDiameter(in.thread_size);
    if (expected <= 0.0) {
        r.add("DFM-FORM-TAP-SIZE", "warning",
              "unknown thread size '" + in.thread_size +
              "' — form-tap pilot chart miss");
    } else {
        auto pilotId = wp.resolve(in.existing_hole_datum);
        if (pilotId) {
            BRepAdaptor_Surface surf(wp.face(*pilotId));
            if (surf.GetType() == GeomAbs_Cylinder) {
                const double pilotDia = 2.0 * surf.Cylinder().Radius();
                if (std::abs(pilotDia - expected) > 0.1) {
                    r.add("DFM-FORM-TAP-DIA", "warning",
                          "pilot diameter " + std::to_string(pilotDia) +
                          " mm does not match FORM-TAP chart for " +
                          in.thread_size + " (expected " +
                          std::to_string(expected) + " mm, larger than the "
                          "cutting-tap pilot because the form tap displaces "
                          "rather than removes material)");
                }
            } else {
                r.add("DFM-FORM-TAP-DATUM", "warning",
                      "existing_hole_datum did not resolve to a cylindrical face");
            }
        } else {
            r.add("DFM-FORM-TAP-DATUM", "warning",
                  "existing_hole_datum unresolved — cannot verify pilot diameter");
        }
    }

    return r;
}

// ── Synthesis (real helical sweep) ───────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "form_tap DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto pilotId = wp.resolve(in.existing_hole_datum);
    const int pilotIdOut = pilotId.value_or(-1);

    gp_Ax1 boreAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    double pilotDia   = 0.0;
    double pilotRad   = 0.0;
    bool   gotAxis    = false;
    if (pilotId) {
        BRepAdaptor_Surface surf(wp.face(*pilotId));
        if (surf.GetType() == GeomAbs_Cylinder) {
            const gp_Cylinder cyl = surf.Cylinder();
            pilotRad = cyl.Radius();
            pilotDia = 2.0 * pilotRad;
            boreAxis = cyl.Axis();
            gotAxis  = true;
        }
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double pitch        = in.pitch_mm;
    const double threadDepthV = pitch * 0.65;
    const double helixLength  = in.thread_depth_mm;

    gp_Dir helixDir = boreAxis.Direction();
    if (helixDir.Z() > 0.0) helixDir = gp_Dir(-helixDir.X(), -helixDir.Y(), -helixDir.Z());

    const gp_Pnt& O = boreAxis.Location();
    gp_Pnt entryPnt = O;
    const double bdZ = boreAxis.Direction().Z();
    if (std::abs(bdZ) > 1e-9) {
        const double t = (zMax - O.Z()) / bdZ;
        entryPnt = gp_Pnt(O.X() + t * boreAxis.Direction().X(),
                          O.Y() + t * boreAxis.Direction().Y(),
                          O.Z() + t * boreAxis.Direction().Z());
    }

    const gp_Ax1 helixAxis(entryPnt, helixDir);

    TopoDS_Shape threadCutter;
    bool         geomSwept = true;
    try {
        if (gotAxis && pilotRad > 0.0 && threadDepthV > 0.0) {
            threadCutter = pr::helicalSweep(helixAxis, pitch, helixLength,
                                            pilotRad, threadDepthV);
        }
    }
    catch (const std::exception& e) {
        spdlog::warn("form_tap: helicalSweep threw ({}); leaving geometry unchanged",
                     e.what());
        threadCutter.Nullify();
        geomSwept = false;
    }

    TopoDS_Shape newShape = wp.shape();
    if (!threadCutter.IsNull()) {
        try {
            newShape = pr::cut(wp.shape(), threadCutter);
        }
        catch (const std::exception& e) {
            spdlog::warn("form_tap: pr::cut failed ({}); leaving workpiece unchanged",
                         e.what());
            newShape = wp.shape();
            geomSwept = false;
        }
    } else {
        geomSwept = false;
    }

    const bool ductile = isDuctileMaterial(wp.material());

    json params = {
        { "pilot_face_id",   pilotIdOut },
        { "thread_size",     in.thread_size },
        { "pitch_mm",        in.pitch_mm },
        { "thread_depth_mm", in.thread_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "thread_size",         in.thread_size },
        { "pitch_mm",            in.pitch_mm },
        { "thread_depth_mm",     in.thread_depth_mm },
        { "pilot_diameter_mm",   pilotDia },
        { "geometry",            geomSwept ? "helical_swept" : "metadata_only" },
        { "process",             "cold_forming_chipless" },
        { "material_ductile",    ductile },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "form_tap";
    tooling.tool_dia_mm       = pilotDia;
    tooling.tool_length_mm    = in.thread_depth_mm * 2.0 + 10.0;
    tooling.tool_material     = "hss-co";   // cobalt HSS or PVD-coated
    tooling.flute_count       = 0;          // form taps are FLUTELESS
    tooling.cutting_speed_sfm = 60.0;       // higher than cutting taps
    tooling.feed_per_tooth_mm = in.pitch_mm;
    tooling.extra["process"]            = "cold_forming";
    tooling.extra["chip_evacuation"]    = "none";
    tooling.extra["requires_lubricant"] = true;
    // Form taps remove no chips; "stock_removed_mm3" is a notional displaced
    // volume to keep the schema uniform with cutting taps.
    {
        const double triArea = 0.5 * threadDepthV * (in.pitch_mm * 0.5);
        const double helLen  = (pilotRad > 0.0 && in.pitch_mm > 0.0)
            ? 2.0 * M_PI * pilotRad * (in.thread_depth_mm / in.pitch_mm)
            : 0.0;
        tooling.stock_removed_mm3 = triArea * helLen;
    }
    tooling.est_cycle_time_s  = std::max(2.0, in.thread_depth_mm / 8.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::form_tap applied: {} pitch={} depth={} ductile={} geom={}",
                  in.thread_size, in.pitch_mm, in.thread_depth_mm, ductile,
                  geomSwept ? "swept" : "metadata");

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.5;   // metadata-only (same geometry as cutting tap)
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::form_tap
