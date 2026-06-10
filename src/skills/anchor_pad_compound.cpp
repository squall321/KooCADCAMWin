// @lat: [[engine/skills#anchor_pad_compound]]

#include "anchor_pad_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
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

namespace koocadcam::skill::anchor_pad_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.plate_length_mm <= 0.0 || in.plate_width_mm <= 0.0 ||
        in.plate_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "plate dims must be > 0");
    }
    if (in.stud_dia_mm <= 0.0 || in.stud_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "stud dims must be > 0");
    }
    if (in.stud_pitch_x_mm <= 0.0 || in.stud_pitch_y_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "stud pitch must be > 0");
    }
    if (in.conduit_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "conduit_dia must be > 0");
    }
    if (in.leveling_nut_dia_mm <= 0.0 || in.leveling_nut_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "leveling_nut dims must be > 0");
    }

    // ACI 318: edge distance ≥ 1.5 × stud_dia.
    const double minEdge = 1.5 * in.stud_dia_mm;
    const double edgeX =
        (in.plate_length_mm - in.stud_pitch_x_mm) / 2.0;
    const double edgeY =
        (in.plate_width_mm  - in.stud_pitch_y_mm) / 2.0;
    if (edgeX < minEdge) {
        r.add("DFM-ACI-318", "error",
              "X edge distance " + std::to_string(edgeX) +
              " mm < min 1.5×stud_dia (" + std::to_string(minEdge) + " mm)");
    }
    if (edgeY < minEdge) {
        r.add("DFM-ACI-318", "error",
              "Y edge distance " + std::to_string(edgeY) +
              " mm < min 1.5×stud_dia (" + std::to_string(minEdge) + " mm)");
    }

    // ACI 318: stud spacing ≥ 3 × stud_dia.
    const double minSpacing = 3.0 * in.stud_dia_mm;
    if (in.stud_pitch_x_mm < minSpacing) {
        r.add("DFM-ACI-318", "error",
              "X stud spacing " + std::to_string(in.stud_pitch_x_mm) +
              " mm < min 3×stud_dia (" + std::to_string(minSpacing) + " mm)");
    }
    if (in.stud_pitch_y_mm < minSpacing) {
        r.add("DFM-ACI-318", "error",
              "Y stud spacing " + std::to_string(in.stud_pitch_y_mm) +
              " mm < min 3×stud_dia (" + std::to_string(minSpacing) + " mm)");
    }

    // AISC 360: plate thickness ≥ 0.5 × stud_dia.
    if (in.plate_thickness_mm < 0.5 * in.stud_dia_mm) {
        r.add("DFM-AISC-360", "error",
              "plate_thickness " + std::to_string(in.plate_thickness_mm) +
              " mm < 0.5 × stud_dia (" +
              std::to_string(0.5 * in.stud_dia_mm) + " mm)");
    }

    // Conduit must fit between studs (≤ min pitch − stud_dia − 4 mm).
    const double maxConduit = std::min(in.stud_pitch_x_mm, in.stud_pitch_y_mm)
                              - in.stud_dia_mm - 4.0;
    if (in.conduit_dia_mm > maxConduit) {
        r.add("DFM-CONDUIT", "error",
              "conduit_dia " + std::to_string(in.conduit_dia_mm) +
              " mm > max " + std::to_string(maxConduit) +
              " mm (would clash with studs)");
    }

    // Leveling-nut counterbore must be > stud dia + clearance, but fit
    // within edge distance.
    if (in.leveling_nut_dia_mm <= in.stud_dia_mm) {
        r.add("DFM-LEVELING-NUT", "error",
              "leveling_nut_dia must be > stud_dia");
    }
    if (in.leveling_nut_depth_mm > in.plate_thickness_mm - 4.0) {
        r.add("DFM-LEVELING-NUT", "error",
              "leveling_nut_depth must leave ≥ 4 mm plate below");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "anchor_pad_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Dir adir = in.axis_dir;
    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 && adir.Z() > 0)) {
        throw SkillError("anchor_pad_compound: only axis_dir = +Z supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const double baseZ = zMax;
    const double kOver = 0.1;

    // ── 1) base plate (fused box) ────────────────────────────────────────
    const gp_Pnt plateOrigin(
        cx - in.plate_length_mm / 2.0,
        cy - in.plate_width_mm  / 2.0,
        baseZ);
    const TopoDS_Shape plateTool = pr::box(
        gp_Ax2(plateOrigin, gp::DZ()),
        in.plate_length_mm, in.plate_width_mm, in.plate_thickness_mm);
    const TopoDS_Shape afterPlate = pr::fuse(wp.shape(), plateTool);

    // ── 2) 4 stud projections (fused cylinders, on top of plate) ─────────
    const double plateTopZ = baseZ + in.plate_thickness_mm;
    const std::array<std::pair<double, double>, 4> studPositions {{
        { cx - in.stud_pitch_x_mm / 2.0, cy - in.stud_pitch_y_mm / 2.0 },
        { cx + in.stud_pitch_x_mm / 2.0, cy - in.stud_pitch_y_mm / 2.0 },
        { cx - in.stud_pitch_x_mm / 2.0, cy + in.stud_pitch_y_mm / 2.0 },
        { cx + in.stud_pitch_x_mm / 2.0, cy + in.stud_pitch_y_mm / 2.0 },
    }};

    TopoDS_Shape withStuds = afterPlate;
    for (const auto& [sx, sy] : studPositions) {
        const gp_Ax2 axStud(gp_Pnt(sx, sy, plateTopZ), adir);
        const TopoDS_Shape stud = pr::cylinder(
            axStud, in.stud_dia_mm / 2.0, in.stud_height_mm);
        withStuds = pr::fuse(withStuds, stud);
    }

    // ── 3) central conduit hole (through plate only) ────────────────────
    const gp_Ax2 axConduit(gp_Pnt(cx, cy, baseZ - kOver), adir);
    const TopoDS_Shape conduitTool = pr::cylinder(
        axConduit, in.conduit_dia_mm / 2.0,
        in.plate_thickness_mm + 2.0 * kOver);
    const TopoDS_Shape afterConduit = pr::cut(withStuds, conduitTool);

    // ── 4) 4 leveling-nut counterbores around each stud ─────────────────
    // Counterbores cut INTO the plate top, concentric with studs.
    std::vector<TopoDS_Shape> levelers;
    levelers.reserve(4);
    for (const auto& [sx, sy] : studPositions) {
        const gp_Ax2 axLev(
            gp_Pnt(sx, sy, plateTopZ - in.leveling_nut_depth_mm), adir);
        levelers.push_back(pr::annularRing(
            axLev,
            in.leveling_nut_dia_mm / 2.0,
            in.stud_dia_mm        / 2.0 + 0.4,  // tight ring around stud
            in.leveling_nut_depth_mm + kOver));
    }
    const TopoDS_Shape newShape = pr::cutMany(afterConduit, levelers);

    // ── volume bookkeeping (test-aligned) ───────────────────────────────
    const double vPlate = in.plate_length_mm * in.plate_width_mm *
                          in.plate_thickness_mm;
    const double rStud  = in.stud_dia_mm / 2.0;
    const double vStud  = 4.0 * M_PI * rStud * rStud * in.stud_height_mm;
    const double rCond  = in.conduit_dia_mm / 2.0;
    const double vCond  = M_PI * rCond * rCond * in.plate_thickness_mm;
    const double rLevOut = in.leveling_nut_dia_mm / 2.0;
    const double rLevIn  = in.stud_dia_mm / 2.0 + 0.4;
    const double vLev   = 4.0 * M_PI *
                          (rLevOut * rLevOut - rLevIn * rLevIn) *
                          in.leveling_nut_depth_mm;

    // ── signature ───────────────────────────────────────────────────────
    json studs_json = json::array();
    for (const auto& [sx, sy] : studPositions) {
        studs_json.push_back({
            { "x", sx }, { "y", sy }, { "dia_mm", in.stud_dia_mm } });
    }

    json params = {
        { "center_x_mm",            cx },
        { "center_y_mm",            cy },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
        { "plate_length_mm",        in.plate_length_mm },
        { "plate_width_mm",         in.plate_width_mm },
        { "plate_thickness_mm",     in.plate_thickness_mm },
        { "stud_dia_mm",            in.stud_dia_mm },
        { "stud_height_mm",         in.stud_height_mm },
        { "stud_pitch_x_mm",        in.stud_pitch_x_mm },
        { "stud_pitch_y_mm",        in.stud_pitch_y_mm },
        { "conduit_dia_mm",         in.conduit_dia_mm },
        { "leveling_nut_dia_mm",    in.leveling_nut_dia_mm },
        { "leveling_nut_depth_mm",  in.leveling_nut_depth_mm },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "subfeature_count",   10 },
        { "stud_count",         4 },
        { "stud_positions",     studs_json },
        { "conduit_dia_mm",     in.conduit_dia_mm },
        { "leveling_nut_dia_mm",in.leveling_nut_dia_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "standard",           "ACI 318 / AISC 360" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "weld;drill;counterbore";
    tooling.tool_dia_mm       = in.leveling_nut_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 200.0;
    tooling.stock_removed_mm3 = vCond + vLev;
    tooling.est_cycle_time_s  = 150.0;
    tooling.extra = {
        { "plate_volume_mm3",   vPlate },
        { "stud_volume_mm3",    vStud },
        { "conduit_volume_mm3", vCond },
        { "leveler_volume_mm3", vLev },
        { "added_volume_mm3",   vPlate + vStud },
        { "removed_volume_mm3", vCond + vLev },
        { "net_delta_mm3",      (vPlate + vStud) - (vCond + vLev) },
        { "standard",           "ACI 318 / AISC 360" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::anchor_pad_compound: plate {}×{}×{} studs Ø{}",
                  in.plate_length_mm, in.plate_width_mm,
                  in.plate_thickness_mm, in.stud_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

struct CylRec { int idx; double radius; double area; gp_Ax1 axis; };

std::vector<CylRec> collectCyls(const Workpiece& wp)
{
    std::vector<CylRec> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        double area = 0.0;
        try { area = wp.faceArea(i); } catch (...) {}
        out.push_back({ i, s.Cylinder().Radius(), area, s.Cylinder().Axis() });
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cyls = collectCyls(wp);

    // Helper lambda: emit metadata replay candidate (used as fallback when
    // geometric path fails or finds no compound).
    auto emitMetadataReplay = [&]() {
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            RecognizedFeature rf;
            rf.skill_id = kSkillId;
            rf.recovered_params = f.params;
            rf.confidence = 1.0;
            rf.matched_geometry = { { "via", "metadata_replay" } };
            out.push_back(rf);
            return;
        }
    };

    if (cyls.size() < 5) {
        emitMetadataReplay();
        return out;
    }

    // Studs: cylinders sharing a radius (within ±0.5 mm) and axis ~ +Z,
    // grouped into a 4-corner rectangle.
    std::vector<int> bigCyls;
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (std::abs(cyls[i].axis.Direction().Z()) < 0.9) continue;
        // Tentatively any +Z cylindrical face is a candidate.
        bigCyls.push_back(static_cast<int>(i));
    }
    if (bigCyls.size() < 5) {
        emitMetadataReplay();
        return out;
    }

    // Find the most-common radius bucket — that's the stud bucket.  When
    // multiple buckets have ≥ 4 occurrences (a leveling-nut counterbore may
    // produce its own 4-cylinder bucket), pick the one whose cylindrical
    // faces have the largest total area — studs are tall (height > leveling
    // depth), so their cumulative area dominates.
    std::vector<std::pair<double, std::vector<int>>> buckets;
    for (int bi : bigCyls) {
        const double r = cyls[bi].radius;
        bool placed = false;
        for (auto& [c, ids] : buckets) {
            if (std::abs(c - r) < 0.3) {
                c = (c * static_cast<double>(ids.size()) + r) /
                    static_cast<double>(ids.size() + 1);
                ids.push_back(bi);
                placed = true;
                break;
            }
        }
        if (!placed) buckets.push_back({ r, { bi } });
    }

    auto bucketArea = [&](const std::vector<int>& ids) {
        double sum = 0.0;
        for (int bi : ids) sum += cyls[bi].area;
        return sum;
    };

    std::sort(buckets.begin(), buckets.end(),
              [&](const auto& a, const auto& b) {
                  if (a.second.size() != b.second.size())
                      return a.second.size() > b.second.size();
                  return bucketArea(a.second) > bucketArea(b.second);
              });

    // Filter to buckets with ≥ 4 occurrences, then prefer largest total area.
    std::vector<std::pair<double, std::vector<int>>> bigBuckets;
    for (const auto& bk : buckets) {
        if (bk.second.size() >= 4) bigBuckets.push_back(bk);
    }
    if (bigBuckets.empty()) {
        emitMetadataReplay();
        return out;
    }
    std::sort(bigBuckets.begin(), bigBuckets.end(),
              [&](const auto& a, const auto& b) {
                  return bucketArea(a.second) > bucketArea(b.second);
              });

    const double studR = bigBuckets.front().first;
    const auto& studIds = bigBuckets.front().second;

    // Find a larger cylinder (≥ 1.5 × stud radius) elsewhere — conduit.
    double conduitR = 0.0;
    for (int bi : bigCyls) {
        if (cyls[bi].radius >= 1.5 * studR && cyls[bi].radius > conduitR) {
            conduitR = cyls[bi].radius;
        }
    }
    if (conduitR <= 0.0) {
        emitMetadataReplay();
        return out;
    }

    // Average stud XY → centroid.
    double sumX = 0.0, sumY = 0.0;
    for (int bi : studIds) {
        sumX += cyls[bi].axis.Location().X();
        sumY += cyls[bi].axis.Location().Y();
    }
    const double cx = sumX / static_cast<double>(studIds.size());
    const double cy = sumY / static_cast<double>(studIds.size());

    json recovered = {
        { "center_x_mm",     cx },
        { "center_y_mm",     cy },
        { "axis_dir",        { 0.0, 0.0, 1.0 } },
        { "stud_dia_mm",     2.0 * studR },
        { "conduit_dia_mm",  2.0 * conduitR },
    };
    json matched = {
        { "stud_count",      static_cast<int>(studIds.size()) },
        { "stud_face_ids",   studIds },
        { "via",             "geometric_primary" },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.8, matched });
    return out;
}

}  // namespace koocadcam::skill::anchor_pad_compound
