// @lat: [[engine/skills#porthole_seal_compound]]

#include "porthole_seal_compound.hpp"

#include "Workpiece.hpp"
#include "_as568_table.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
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

namespace koocadcam::skill::porthole_seal_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.porthole_dia_mm <= 0.0 || in.bolt_pcd_mm <= 0.0 ||
        in.plate_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "porthole_seal_compound: dimensions must be > 0");
    }

    if (in.bolt_count != 8 && in.bolt_count != 12 && in.bolt_count != 16) {
        r.add("DFM-MARINE-BOLT", "error",
              "bolt_count " + std::to_string(in.bolt_count) +
              " not in marine porthole standard set {8, 12, 16}");
    }

    const auto* asSpec = as568::findDash(in.o_ring_size_key);
    if (!asSpec) {
        r.add("DFM-AS568", "error",
              "o_ring_size_key '" + in.o_ring_size_key +
              "' not present in central _as568_table.hpp");
    }

    const auto* mSpec = thread_table::findMetric(in.bolt_thread_size_key);
    if (!mSpec) {
        r.add("DFM-M-THREAD", "error",
              "bolt_thread_size_key '" + in.bolt_thread_size_key +
              "' not present in central _iso_thread_table.hpp");
    }

    // Bolt PCD must clear the O-ring groove on the inside and leave material
    // on the outside.
    if (asSpec && mSpec) {
        const double grooveOuterDia =
            in.porthole_dia_mm + 2.0 * asSpec->groove_width_mm + 4.0;
        if (in.bolt_pcd_mm - mSpec->clearance_medium_mm < grooveOuterDia) {
            r.add("DFM-MARINE-PCD", "error",
                  "bolt_pcd_mm too small — bolts would clash with O-ring groove");
        }
    }

    if (in.plate_thickness_mm < 3.0) {
        r.add("DFM-MARINE-PLATE", "error",
              "plate_thickness_mm < 3.0 mm (marine hull plate minimum)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// SEQUENTIAL build chain (CRITICAL):
//   step A: cut central porthole through-bore.
//   step B: cut O-ring annular face groove (centred on porthole, on entry face).
//   step C: for i in 0..N-1, build bolt cylinder using SetRotation, then
//           call pr::cut(prev, bolt) — DO NOT use pr::cutMany with the
//           bolt compound (overlap heuristics in OCCT 8.0 silently no-op).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "porthole_seal_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Dir adir = in.axis_dir;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;

    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 &&
          adir.Z() > 0)) {
        throw SkillError("porthole_seal_compound: only axis_dir = +Z is supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ  = zMax;
    const double thk   = in.plate_thickness_mm;
    const double kOver = 0.1;

    const auto* asSpec = as568::findDash(in.o_ring_size_key);
    const auto* mSpec  = thread_table::findMetric(in.bolt_thread_size_key);

    const double boreR    = in.porthole_dia_mm / 2.0;
    const double grooveD  = asSpec->groove_depth_mm;
    const double grooveW  = asSpec->groove_width_mm;
    const double grooveID = in.porthole_dia_mm + 2.0;       // sit ~1 mm outside bore
    const double grooveOD = grooveID + 2.0 * grooveW;
    const double boltDia  = mSpec->clearance_medium_mm;
    const double pcdR     = in.bolt_pcd_mm / 2.0;

    // ── A) central porthole through-bore ────────────────────────────────
    const gp_Ax2 axBore(gp_Pnt(cx, cy, topZ - thk - kOver), adir);
    const TopoDS_Shape boreTool = pr::cylinder(
        axBore, boreR, thk + 2.0 * kOver);
    const TopoDS_Shape afterBore = pr::cut(wp.shape(), boreTool);

    // ── B) O-ring annular face groove on entry (top) face ───────────────
    const gp_Ax2 axGroove(gp_Pnt(cx, cy, topZ - grooveD), adir);
    const TopoDS_Shape grooveTool = pr::annularRing(
        axGroove, grooveOD / 2.0, grooveID / 2.0, grooveD + kOver);
    const TopoDS_Shape afterGroove = pr::cut(afterBore, grooveTool);

    // ── C) N bolt holes — SEQUENTIAL cuts via gp_Trsf::SetRotation ──────
    TopoDS_Shape current = afterGroove;
    const gp_Ax1 spinAxis(gp_Pnt(cx, cy, topZ), adir);
    // Build the i=0 bolt cylinder, then rotate copy by k·dθ for each k.
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        const double bx = cx + pcdR * std::cos(theta);
        const double by = cy + pcdR * std::sin(theta);
        const gp_Ax2 axBolt(gp_Pnt(bx, by, topZ - thk - kOver), adir);
        const TopoDS_Shape boltTool = pr::cylinder(
            axBolt, boltDia / 2.0, thk + 2.0 * kOver);
        current = pr::cut(current, boltTool);
    }
    (void)spinAxis;
    const TopoDS_Shape newShape = current;

    // ── signature ────────────────────────────────────────────────────────
    json bolts_json = json::array();
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        bolts_json.push_back({
            { "x", cx + pcdR * std::cos(theta) },
            { "y", cy + pcdR * std::sin(theta) },
            { "dia_mm", boltDia },
        });
    }

    const int subfeatureCount = 2 + in.bolt_count;  // bore + groove + N bolts

    json params = {
        { "center_x_mm",        cx },
        { "center_y_mm",        cy },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "porthole_dia_mm",    in.porthole_dia_mm },
        { "o_ring_size_key",    in.o_ring_size_key },
        { "bolt_count",         in.bolt_count },
        { "bolt_pcd_mm",        in.bolt_pcd_mm },
        { "bolt_thread_size_key", in.bolt_thread_size_key },
        { "plate_thickness_mm", in.plate_thickness_mm },
    };

    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "marine_feature_type", "porthole_with_seal" },
        { "subfeature_count",    subfeatureCount },
        { "porthole_dia_mm",     in.porthole_dia_mm },
        { "o_ring_dash",         in.o_ring_size_key },
        { "o_ring_groove_id_mm", grooveID },
        { "o_ring_groove_od_mm", grooveOD },
        { "o_ring_groove_depth_mm", grooveD },
        { "bolt_count",          in.bolt_count },
        { "bolt_pcd_mm",         in.bolt_pcd_mm },
        { "bolt_thread_size",    in.bolt_thread_size_key },
        { "bolt_hole_dia_mm",    boltDia },
        { "bolt_positions",      bolts_json },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "standard",            "AS568 + ISO 273 + marine practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type = "drill;groove_cutter";
    tooling.tool_dia_mm = std::max(boltDia, in.porthole_dia_mm);
    tooling.tool_material = "carbide";
    tooling.flute_count = 4;
    tooling.cutting_speed_sfm = 220.0;
    const double vBore   = M_PI * boreR * boreR * thk;
    const double vGroove = M_PI *
        ((grooveOD * 0.5) * (grooveOD * 0.5) - (grooveID * 0.5) * (grooveID * 0.5)) *
        grooveD;
    const double vBolts  = static_cast<double>(in.bolt_count) *
        M_PI * (boltDia * 0.5) * (boltDia * 0.5) * thk;
    tooling.stock_removed_mm3 = vBore + vGroove + vBolts;
    tooling.est_cycle_time_s  = 80.0 + 5.0 * static_cast<double>(in.bolt_count);
    tooling.extra = {
        { "removed_volume_mm3", vBore + vGroove + vBolts },
        { "standard",           "AS568 + ISO 273 + marine porthole practice" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::porthole_seal_compound: dia={} bolts={} pcd={} faces {}→{}",
                  in.porthole_dia_mm, in.bolt_count, in.bolt_pcd_mm,
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

bool parallelAxis(const gp_Ax1& a, const gp_Ax1& b)
{
    return std::abs(a.Direction().Dot(b.Direction())) > 0.999;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay first.
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
    if (cyls.size() < 9) return out;  // 1 bore + ≥ 8 bolts

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double midX = 0.5 * (xMin + xMax);
    const double midY = 0.5 * (yMin + yMax);

    // Look for one big central +Z cylinder = porthole bore.
    for (const auto& central : cyls) {
        if (std::abs(central.axis.Direction().Z()) < 0.9) continue;
        if (std::abs(central.axis.Location().X() - midX) > 8.0) continue;
        if (std::abs(central.axis.Location().Y() - midY) > 8.0) continue;
        if (central.radius < 50.0) continue;  // porthole ≥ 100 mm dia

        // Collect bolt cylinders on a single PCD.
        std::vector<const CylRec*> boltCands;
        for (const auto& c : cyls) {
            if (c.faceIdx == central.faceIdx) continue;
            if (!parallelAxis(c.axis, central.axis)) continue;
            if (c.radius >= central.radius) continue;
            const double dx = c.axis.Location().X() - central.axis.Location().X();
            const double dy = c.axis.Location().Y() - central.axis.Location().Y();
            const double r  = std::hypot(dx, dy);
            if (r < central.radius + 5.0) continue;  // skip groove edges
            boltCands.push_back(&c);
        }

        if (boltCands.size() < 8) continue;

        // Group by PCD radius.
        std::vector<double> rs;
        for (auto* b : boltCands) {
            const double dx = b->axis.Location().X() - central.axis.Location().X();
            const double dy = b->axis.Location().Y() - central.axis.Location().Y();
            rs.push_back(std::hypot(dx, dy));
        }
        std::sort(rs.begin(), rs.end());
        std::vector<std::pair<double, int>> hist;
        for (double rv : rs) {
            bool placed = false;
            for (auto& [c, cnt] : hist) {
                if (std::abs(c - rv) < 2.0) {
                    c = (c * static_cast<double>(cnt) + rv) /
                        static_cast<double>(cnt + 1);
                    ++cnt;
                    placed = true;
                    break;
                }
            }
            if (!placed) hist.push_back({ rv, 1 });
        }
        std::sort(hist.begin(), hist.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        if (hist.empty() || hist.front().second < 8) continue;

        const double pcdR = hist.front().first;
        const int boltCount = hist.front().second;

        double sumR = 0.0; int cnt = 0;
        for (auto* b : boltCands) {
            const double dx = b->axis.Location().X() - central.axis.Location().X();
            const double dy = b->axis.Location().Y() - central.axis.Location().Y();
            const double rv = std::hypot(dx, dy);
            if (std::abs(rv - pcdR) < 2.0) { sumR += b->radius; ++cnt; }
        }
        if (cnt == 0) continue;
        const double boltDia = 2.0 * (sumR / static_cast<double>(cnt));

        double conf = 0.85;
        if (boltCount < 8 || boltCount > 16) conf -= 0.2;

        json recovered = {
            { "center_x_mm",      central.axis.Location().X() },
            { "center_y_mm",      central.axis.Location().Y() },
            { "axis_dir",         { 0.0, 0.0, 1.0 } },
            { "porthole_dia_mm",  2.0 * central.radius },
            { "bolt_count",       boltCount },
            { "bolt_pcd_mm",      2.0 * pcdR },
            { "bolt_hole_dia_mm", boltDia },
        };
        json matched = {
            { "central_face_id",     central.faceIdx },
            { "bolt_cylinder_count", boltCount },
            { "via",                 "geometric_primary_pcd_detection" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }

    return out;
}

}  // namespace koocadcam::skill::porthole_seal_compound
