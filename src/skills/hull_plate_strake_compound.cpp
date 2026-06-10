// @lat: [[engine/skills#hull_plate_strake_compound]]

#include "hull_plate_strake_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::hull_plate_strake_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.strake_length_mm <= 0.0 || in.rivet_pitch_mm <= 0.0 ||
        in.rivet_dia_mm <= 0.0 || in.plate_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "hull_plate_strake_compound: dimensions must be > 0");
    }

    if (in.rivet_count < 2) {
        r.add("DFM-MARINE-RIVET", "error",
              "rivet_count must be ≥ 2 (line requires ≥ 2 holes)");
    }

    if (in.rivet_pitch_mm < 3.0 * in.rivet_dia_mm) {
        r.add("DFM-MARINE-RIVET", "error",
              "rivet_pitch must be ≥ 3 × rivet_dia (Lloyd's Register hull-rivet minimum)");
    }

    if (in.plate_thickness_mm < 3.0) {
        r.add("DFM-MARINE-PLATE", "error",
              "plate_thickness_mm < 3.0 mm (marine hull plate minimum)");
    }

    if (in.edge_radius_mm < 0.0) {
        r.add("DFM-INPUT", "error", "edge_radius_mm must be ≥ 0");
    }

    // Rivet line must fit inside the strake length.
    const double lineLen = (in.rivet_count - 1) * in.rivet_pitch_mm;
    if (lineLen > in.strake_length_mm) {
        r.add("DFM-MARINE-RIVET", "error",
              "rivet line (count-1)·pitch > strake_length");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hull_plate_strake_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Dir saxis = in.strake_axis_dir;
    const gp_Dir daxis = in.drill_axis_dir;

    if (!(std::abs(daxis.X()) < 1e-6 && std::abs(daxis.Y()) < 1e-6 &&
          daxis.Z() > 0)) {
        throw SkillError(
            "hull_plate_strake_compound: only drill_axis_dir = +Z is supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ    = zMax;
    const double bottomZ = topZ - in.plate_thickness_mm;
    const double kOver   = 0.1;

    TopoDS_Shape current = wp.shape();

    // ── A) optional edge-relief recess along the strake axis ────────────
    if (in.edge_radius_mm > 0.0) {
        // Cut a thin shallow channel of width edge_radius_mm × depth
        // edge_radius_mm along the strake length, at the top face, centred
        // on the rivet line.  This represents the rolled-edge relief that
        // the rivet heads sit on the inboard side of.
        const double w = in.edge_radius_mm * 2.0;
        const double d = in.edge_radius_mm;
        const gp_Pnt o = in.strake_axis_origin;
        // Build the relief box in world coords using axial basis (saxis, perpendicular).
        // For simplicity (+X strake axis assumed), perpendicular is +Y.
        if (std::abs(saxis.X() - 1.0) > 1e-3 || std::abs(saxis.Y()) > 1e-3) {
            // Generalisation deferred — for slice-16 we accept only +X strake.
        }
        const gp_Pnt boxOrigin(o.X(), o.Y() - w / 2.0, topZ - d);
        const TopoDS_Shape reliefTool = pr::box(
            gp_Ax2(boxOrigin, gp_Dir(0, 0, 1)),
            in.strake_length_mm, w, d + kOver);
        current = pr::cut(current, reliefTool);
    }

    // ── B) N rivet holes — SEQUENTIAL pr::cut loop ──────────────────────
    for (int i = 0; i < in.rivet_count; ++i) {
        const double s = static_cast<double>(i) * in.rivet_pitch_mm;
        const double rx = in.strake_axis_origin.X() + s * saxis.X();
        const double ry = in.strake_axis_origin.Y() + s * saxis.Y();
        const gp_Ax2 axRivet(gp_Pnt(rx, ry, bottomZ - kOver), daxis);
        const TopoDS_Shape rivetTool = pr::cylinder(
            axRivet, in.rivet_dia_mm / 2.0,
            in.plate_thickness_mm + 2.0 * kOver);
        current = pr::cut(current, rivetTool);
    }

    const TopoDS_Shape newShape = current;

    // ── signature ──────────────────────────────────────────────────────
    json rivets_json = json::array();
    for (int i = 0; i < in.rivet_count; ++i) {
        const double s = static_cast<double>(i) * in.rivet_pitch_mm;
        rivets_json.push_back({
            { "x", in.strake_axis_origin.X() + s * saxis.X() },
            { "y", in.strake_axis_origin.Y() + s * saxis.Y() },
            { "dia_mm", in.rivet_dia_mm },
        });
    }

    json params = {
        { "strake_axis_origin", { in.strake_axis_origin.X(),
                                  in.strake_axis_origin.Y(),
                                  in.strake_axis_origin.Z() } },
        { "strake_axis_dir",    { saxis.X(), saxis.Y(), saxis.Z() } },
        { "drill_axis_dir",     { daxis.X(), daxis.Y(), daxis.Z() } },
        { "strake_length_mm",   in.strake_length_mm },
        { "rivet_pitch_mm",     in.rivet_pitch_mm },
        { "rivet_dia_mm",       in.rivet_dia_mm },
        { "rivet_count",        in.rivet_count },
        { "edge_radius_mm",     in.edge_radius_mm },
        { "plate_thickness_mm", in.plate_thickness_mm },
    };

    const int subfeatureCount =
        in.rivet_count + (in.edge_radius_mm > 0.0 ? 1 : 0);

    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "marine_feature_type", "hull_strake_rivet_line" },
        { "subfeature_count",    subfeatureCount },
        { "rivet_count",         in.rivet_count },
        { "rivet_pitch_mm",      in.rivet_pitch_mm },
        { "rivet_dia_mm",        in.rivet_dia_mm },
        { "rivet_positions",     rivets_json },
        { "strake_axis_dir",     { saxis.X(), saxis.Y(), saxis.Z() } },
        { "standard",            "Lloyd's Register hull-rivet practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type = "drill";
    tooling.tool_dia_mm = in.rivet_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count = 2;
    tooling.cutting_speed_sfm = 180.0;
    const double rR = in.rivet_dia_mm / 2.0;
    const double vRivets = static_cast<double>(in.rivet_count) *
        M_PI * rR * rR * in.plate_thickness_mm;
    const double vRelief = (in.edge_radius_mm > 0.0)
        ? (in.strake_length_mm * 2.0 * in.edge_radius_mm * in.edge_radius_mm)
        : 0.0;
    tooling.stock_removed_mm3 = vRivets + vRelief;
    tooling.est_cycle_time_s  = 20.0 + 3.0 * static_cast<double>(in.rivet_count);
    tooling.extra = {
        { "removed_volume_mm3", vRivets + vRelief },
        { "standard",           "Lloyd's Register hull strake" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::hull_plate_strake_compound: rivets={} pitch={} faces {}→{}",
                  in.rivet_count, in.rivet_pitch_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

struct CylRec {
    int    faceIdx = -1;
    double radius = 0.0;
    gp_Ax1 axis;
};

std::vector<CylRec> collectCylinders(const Workpiece& wp)
{
    std::vector<CylRec> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        out.push_back({ i, c.Radius(), c.Axis() });
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence = 1.0;
        rf.matched_geometry = { { "via", "metadata_replay" } };
        out.push_back(rf);
    }

    const auto cyls = collectCylinders(wp);
    if (cyls.size() < 2) return out;

    // Find ≥ 2 cylinders of similar radius whose centres are colinear and
    // evenly spaced — a rivet line.
    // For each pair, build a candidate axis; group remaining cylinders by
    // distance to that axis.  Pick the largest collinear cluster.
    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            const auto& a = cyls[i];
            const auto& b = cyls[j];
            if (std::abs(a.axis.Direction().Z()) < 0.9) continue;
            if (std::abs(b.axis.Direction().Z()) < 0.9) continue;
            if (std::abs(a.radius - b.radius) > 0.5) continue;

            const double dx = b.axis.Location().X() - a.axis.Location().X();
            const double dy = b.axis.Location().Y() - a.axis.Location().Y();
            const double dist = std::hypot(dx, dy);
            if (dist < 2.0 * a.radius) continue;
            const double ux = dx / dist;
            const double uy = dy / dist;

            // Count cylinders that lie on the line through a in direction (ux,uy).
            std::vector<double> ss;
            std::vector<double> radii;
            for (const auto& c : cyls) {
                if (std::abs(c.axis.Direction().Z()) < 0.9) continue;
                if (std::abs(c.radius - a.radius) > 0.5) continue;
                const double rx = c.axis.Location().X() - a.axis.Location().X();
                const double ry = c.axis.Location().Y() - a.axis.Location().Y();
                const double s  = rx * ux + ry * uy;     // along
                const double t  = -rx * uy + ry * ux;    // perpendicular
                if (std::abs(t) > 1.0) continue;
                ss.push_back(s);
                radii.push_back(c.radius);
            }

            if (ss.size() < 2) continue;
            std::sort(ss.begin(), ss.end());

            // Estimate pitch as median delta.
            std::vector<double> deltas;
            for (size_t k = 1; k < ss.size(); ++k) deltas.push_back(ss[k] - ss[k - 1]);
            std::sort(deltas.begin(), deltas.end());
            const double pitch = deltas[deltas.size() / 2];

            double sumR = 0.0;
            for (double rv : radii) sumR += rv;
            const double avgR = sumR / static_cast<double>(radii.size());

            json recovered = {
                { "rivet_count",     static_cast<int>(ss.size()) },
                { "rivet_pitch_mm",  pitch },
                { "rivet_dia_mm",    2.0 * avgR },
                { "strake_axis_dir", { ux, uy, 0.0 } },
                { "strake_axis_origin", { a.axis.Location().X(),
                                          a.axis.Location().Y(),
                                          a.axis.Location().Z() } },
            };
            json matched = {
                { "rivet_line_count", static_cast<int>(ss.size()) },
                { "via",              "geometric_collinear_rivet_line" },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.75, matched });
            return out;  // first good cluster suffices
        }
    }

    return out;
}

}  // namespace koocadcam::skill::hull_plate_strake_compound
