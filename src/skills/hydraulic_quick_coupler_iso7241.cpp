// @lat: [[engine/skills#hydraulic_quick_coupler_iso7241]]

#include "hydraulic_quick_coupler_iso7241.hpp"

#include "Workpiece.hpp"
#include "_as568_table.hpp"
#include "_hydraulic_ports.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::hydraulic_quick_coupler_iso7241 {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hyd_ports;
namespace as = koocadcam::skill::as568;
using nlohmann::json;

namespace {

constexpr double kOver = 0.1;

struct Iso7241Spec {
    const char* dash_size;          // "-4", "-6", "-8"
    double      port_dia_mm;        // central port bore for male tip
    double      spotface_dia_mm;    // body shoulder counterbore
    double      spotface_depth_mm;
    const char* as568_dash;         // AS568 ring used for face seal
};

// ISO 7241-A flat-face: dash -4 / -6 / -8 correspond to 1/4" / 3/8" / 1/2"
// nominal flow; the captive face seal at the mating flat is an AS568 ring
// sized for that bore (typically -111/-114/-116 series).
constexpr Iso7241Spec kIso7241[] {
    { "-4",  6.35,  14.0, 3.0, "-111" },
    { "-6",  9.53,  20.0, 3.5, "-114" },
    { "-8", 12.70,  24.0, 4.0, "-116" },
};

const Iso7241Spec* findDash(const std::string& dash)
{
    for (const auto& s : kIso7241) if (dash == s.dash_size) return &s;
    return nullptr;
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const Iso7241Spec* iso = findDash(in.dash_size);
    if (!iso) {
        r.add("DFM-DASH", "error",
              "hydraulic_quick_coupler_iso7241: dash_size '" + in.dash_size +
              "' invalid (must be -4 | -6 | -8 per ISO 7241-A)");
    }
    if (in.thread_size_key.empty()) {
        r.add("DFM-THREAD", "error",
              "hydraulic_quick_coupler_iso7241: thread_size_key is empty");
    } else if (!ht::findBspG(in.thread_size_key)
               && !ht::findNpt(in.thread_size_key)) {
        r.add("DFM-THREAD", "error",
              "hydraulic_quick_coupler_iso7241: thread_size_key '" +
              in.thread_size_key +
              "' not in central BSP G or NPT table");
    }
    if (iso && !as::findDash(iso->as568_dash)) {
        r.add("DFM-AS568", "error",
              "hydraulic_quick_coupler_iso7241: derived AS568 dash '" +
              std::string(iso->as568_dash) + "' missing from central table");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hydraulic_quick_coupler_iso7241 DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const Iso7241Spec* iso = findDash(in.dash_size);
    if (!iso) throw SkillError("hydraulic_quick_coupler_iso7241: dash lookup failed");
    const as::DashSpec* ring = as::findDash(iso->as568_dash);
    if (!ring) throw SkillError("hydraulic_quick_coupler_iso7241: AS568 lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Central port bore ──────────────────────────────────────────
    const double portR = iso->port_dia_mm / 2.0;
    const double portDepth = std::max(iso->spotface_depth_mm + 5.0,
                                      (zMax - zMin) * 0.6);
    const gp_Pnt portStart(cx, cy, topZ - portDepth);
    const gp_Ax2 portAx(portStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(portAx, portR, portDepth + kOver));

    // ── 2) AS568 face groove (annular ring) ──────────────────────────
    // Groove ID matches port_dia + 2*radial_inset; groove OD = ID + width.
    const double groovId = (ring->inner_dia_mm + ring->cross_section_mm) - ring->groove_width_mm / 2.0;
    const double groovOd = groovId + ring->groove_width_mm;
    const double groovDepth = ring->groove_depth_mm;
    const gp_Pnt groovStart(cx, cy, topZ - groovDepth);
    const gp_Ax2 groovAx(groovStart, gp::DZ());
    // Use annularRing primitive (outer cyl - inner cyl).
    const TopoDS_Shape grooveTool = pr::annularRing(
        groovAx, groovOd / 2.0, groovId / 2.0, groovDepth + kOver);
    current = pr::cut(current, grooveTool);

    // ── 3) Spotface counterbore for body shoulder ────────────────────
    const gp_Pnt spotStart(cx, cy, topZ - iso->spotface_depth_mm);
    const gp_Ax2 spotAx(spotStart, gp::DZ());
    const TopoDS_Shape spotTool = pr::cylinder(
        spotAx, iso->spotface_dia_mm / 2.0, iso->spotface_depth_mm + kOver);
    current = pr::cut(current, spotTool);

    const double vPort = M_PI * portR * portR * portDepth;
    const double vGroove = M_PI * ((groovOd*groovOd - groovId*groovId) / 4.0) * groovDepth;
    const double vSpot = M_PI * std::pow(iso->spotface_dia_mm / 2.0, 2)
                         * iso->spotface_depth_mm
                         - M_PI * portR * portR * iso->spotface_depth_mm;
    const double volRemoved = vPort + vGroove + std::max(0.0, vSpot);

    json params = {
        { "center_xy",       { in.center_xy.X(),
                               in.center_xy.Y(),
                               in.center_xy.Z() } },
        { "dash_size",       in.dash_size },
        { "thread_size_key", in.thread_size_key },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "ag_feature_type",            "iso7241_flat_face_quick_coupler" },
        { "subfeature_count",           3 },
        { "dash_size",                  in.dash_size },
        { "thread_size_key",            in.thread_size_key },
        { "derived_port_dia_mm",        iso->port_dia_mm },
        { "derived_spotface_dia_mm",    iso->spotface_dia_mm },
        { "derived_as568_dash",         iso->as568_dash },
        { "derived_groove_id_mm",       groovId },
        { "derived_groove_od_mm",       groovOd },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 7241-A" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;groove_mill;counterbore";
    tooling.tool_dia_mm       = iso->port_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(30.0, 40.0);
    tooling.extra = {
        { "ag_application", "hydraulic_quick_coupler" },
        { "standard",       "ISO 7241-A" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::hydraulic_quick_coupler_iso7241: dash={} thread={}",
                  in.dash_size, in.thread_size_key);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric: count concentric cylindrical faces — port + spotface +
    // annular groove walls.  ISO 7241 leaves at least 3 cyl faces near
    // the port axis.
    int portCyls = 0;
    int grooveCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 2.0 && radius <= 8.0) ++portCyls;
            else if (radius > 8.0 && radius <= 14.0) ++grooveCyls;
        } catch (...) {}
    }
    if (portCyls >= 1 && grooveCyls >= 1) {
        json recovered = { { "dash_size",       "-6" },
                           { "thread_size_key", "G3/8" } };
        json matched   = { { "source", "geometric_port_groove_pattern" },
                           { "port_cyls",   portCyls },
                           { "groove_cyls", grooveCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::hydraulic_quick_coupler_iso7241
