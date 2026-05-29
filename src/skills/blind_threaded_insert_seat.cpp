// @lat: [[engine/skills#blind_threaded_insert_seat]]

#include "blind_threaded_insert_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace koocadcam::skill::blind_threaded_insert_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct InsertEntry {
    const char* size;
    double      od_mm;
    double      head_dia_mm;
    double      engagement_mm;
    double      binder_reservoir_mm;
};

constexpr std::array<InsertEntry, 7> kTable {{
    { "M2",   3.5,  4.5,  4.0,  0.5 },
    { "M2.5", 4.0,  5.0,  4.5,  0.5 },
    { "M3",   4.5,  5.5,  5.5,  0.7 },
    { "M4",   5.5,  7.0,  7.0,  0.8 },
    { "M5",   7.0,  9.0,  8.5,  1.0 },
    { "M6",   9.0,  11.0, 10.0, 1.2 },
    { "M8",   11.0, 13.5, 13.0, 1.5 },
}};

const InsertEntry* lookupEntry(const std::string& s) {
    for (const auto& e : kTable) if (s == e.size) return &e;
    return nullptr;
}

constexpr double kChamferDepth = 0.3;

}  // namespace

double insertOuterDiameterFor(const std::string& s)
{ const auto* e = lookupEntry(s); return e ? e->od_mm : 0.0; }

double insertHeadDiameterFor(const std::string& s)
{ const auto* e = lookupEntry(s); return e ? e->head_dia_mm : 0.0; }

double insertEngagementFor(const std::string& s)
{ const auto* e = lookupEntry(s); return e ? e->engagement_mm : 0.0; }

double binderReservoirFor(const std::string& s)
{ const auto* e = lookupEntry(s); return e ? e->binder_reservoir_mm : 0.0; }

double materialSlipFor(const std::string& m)
{
    if (m == "metal")          return 0.0;
    if (m == "thermoplastic")  return 0.1;
    if (m == "thermoset")      return 0.2;
    if (m == "composite_fbg")  return 0.3;
    return -1.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.insert_size.empty()) {
        r.add("DFM-INSERT-SIZE", "error",
              "blind_threaded_insert_seat: insert_size is empty");
        return r;
    }
    const auto* e = lookupEntry(in.insert_size);
    if (!e) {
        r.add("DFM-INSERT-SIZE", "error",
              "blind_threaded_insert_seat: unknown insert_size '" +
              in.insert_size +
              "' (supported: M2/M2.5/M3/M4/M5/M6/M8)");
        return r;
    }

    const double slip = materialSlipFor(in.insert_material);
    if (slip < 0.0) {
        r.add("DFM-INSERT-MATERIAL", "error",
              "blind_threaded_insert_seat: unknown insert_material '" +
              in.insert_material +
              "' (supported: metal/thermoplastic/thermoset/composite_fbg)");
    }

    const double pilotDia = e->od_mm + std::max(0.0, slip);
    if (pilotDia < 0.8) {
        r.add("DFM-002", "error",
              "blind_threaded_insert_seat: derived pilot " +
              std::to_string(pilotDia) + " < 0.8 mm");
    }

    // Required depth: head_seat (~kChamferDepth + 0.5) + engagement + binder.
    double binderDepth = e->binder_reservoir_mm;
    if (in.insert_material == "composite_fbg") binderDepth *= 1.5;
    const double totalDepth = kChamferDepth + e->engagement_mm + binderDepth;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > 0.0 && thickness < totalDepth + 1.0) {
        r.add("DFM-INSERT-THICKNESS", "error",
              "blind_threaded_insert_seat: stock thickness " +
              std::to_string(thickness) +
              " mm < required total depth + 1 mm (" +
              std::to_string(totalDepth + 1.0) + " mm)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "blind_threaded_insert_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* e = lookupEntry(in.insert_size);
    const double slip = materialSlipFor(in.insert_material);

    const double pilotDia    = e->od_mm + slip;
    const double headDia     = e->head_dia_mm + slip;
    const double engagement  = e->engagement_mm;
    double binderDepth = e->binder_reservoir_mm;
    if (in.insert_material == "composite_fbg") binderDepth *= 1.5;
    const double headSeatDepth = 0.5;  // head sits just sub-flush
    const double binderDia     = pilotDia + 0.4;  // 0.2 mm radial relief

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("blind_threaded_insert_seat: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt pilotStart(
        in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    gp_Pnt entryPlanePoint(pilotStart);

    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        const double entryZ = (adir.Z() < 0) ? zMax : zMin;
        pilotStart      = gp_Pnt(in.position_x_mm, in.position_y_mm,
                                 entryZ - adir.Z() * kOverhang);
        entryPlanePoint = gp_Pnt(in.position_x_mm, in.position_y_mm, entryZ);
    }

    const gp_Ax2 pilotAx (pilotStart,      adir);
    const gp_Ax2 entryAx (entryPlanePoint, adir);

    // ── Sub-feature 1: pilot drill (engagement depth) ────────────────────
    const double pilotDepth = headSeatDepth + engagement;
    const TopoDS_Shape pilotTool =
        pr::cylinder(pilotAx, pilotDia / 2.0, pilotDepth + kOverhang);

    // ── Sub-feature 2: head counterbore (small dia bump at entry) ────────
    const TopoDS_Shape headTool =
        pr::cylinder(entryAx, headDia / 2.0, headSeatDepth + kOverhang);

    // ── Sub-feature 3: chamfer (45° entry) ───────────────────────────────
    const double chamferTopR = headDia / 2.0 + kChamferDepth;
    const double chamferBotR = headDia / 2.0;
    const TopoDS_Shape chamferTool =
        pr::coneFrustum(entryAx, chamferTopR, chamferBotR, kChamferDepth);

    // ── Sub-feature 4: bottom binder reservoir (deeper, wider) ───────────
    const gp_Pnt binderStart(
        entryPlanePoint.X() + adir.X() * pilotDepth,
        entryPlanePoint.Y() + adir.Y() * pilotDepth,
        entryPlanePoint.Z() + adir.Z() * pilotDepth);
    const gp_Ax2 binderAx(binderStart, adir);
    const TopoDS_Shape binderTool =
        pr::cylinder(binderAx, binderDia / 2.0, binderDepth + kOverhang);

    // Fuse all four concentric cutters.
    TopoDS_Shape fused = pr::fuse(pilotTool, headTool);
    fused = pr::fuse(fused, chamferTool);
    fused = pr::fuse(fused, binderTool);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "insert_size",     in.insert_size },
        { "insert_material", in.insert_material },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },
        { "insert_size",         in.insert_size },
        { "insert_material",     in.insert_material },
        { "pilot_dia_mm",        pilotDia },
        { "head_dia_mm",         headDia },
        { "head_seat_depth_mm",  headSeatDepth },
        { "engagement_mm",       engagement },
        { "binder_dia_mm",       binderDia },
        { "binder_depth_mm",     binderDepth },
        { "chamfer_depth_mm",    kChamferDepth },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "drill;counterbore;chamfer;endmill";
    tooling.tool_dia_mm = headDia;
    tooling.tool_length_mm = (pilotDepth + binderDepth) * 1.5 + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double pilotR  = pilotDia  / 2.0;
    const double headR   = headDia   / 2.0;
    const double binderR = binderDia / 2.0;
    tooling.stock_removed_mm3 =
        M_PI * pilotR * pilotR * pilotDepth +
        M_PI * (headR * headR - pilotR * pilotR) * headSeatDepth +
        M_PI * (binderR * binderR - pilotR * pilotR) * binderDepth;
    tooling.est_cycle_time_s  = std::max(2.0,
        (pilotDepth + binderDepth) / 40.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },       { "tool_dia_mm", pilotDia },
              { "depth_mm",  pilotDepth } },
            { { "tool_type", "counterbore" }, { "tool_dia_mm", headDia },
              { "depth_mm",  headSeatDepth } },
            { { "tool_type", "chamfer" },     { "tool_dia_mm", headDia + 2.0 * kChamferDepth },
              { "depth_mm",  kChamferDepth } },
            { { "tool_type", "endmill" },     { "tool_dia_mm", binderDia },
              { "depth_mm",  binderDepth },
              { "operation", "binder_reservoir" } },
        } },
        { "insert_standard", "Tappex_Trisert_or_SelfClinch" },
        { "binder_required", (in.insert_material == "thermoset" ||
                              in.insert_material == "composite_fbg") },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::blind_threaded_insert_seat applied: {} mat={} pilot={}×{}",
                  in.insert_size, in.insert_material, pilotDia, pilotDepth);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::blind_threaded_insert_seat
