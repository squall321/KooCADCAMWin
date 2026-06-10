// @lat: [[engine/skills#jic_flare_port_seat]]

#include "jic_flare_port_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace koocadcam::skill::jic_flare_port_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// 37° per side flare half-angle (SAE J514). Included = 74°.
constexpr double kJicHalfAngleDeg = 37.0;

namespace {

struct JicEntry {
    const char* size;
    double      tube_od_mm;
    double      thread_major_dia_mm;
    double      thread_pitch_mm;       // = 25.4 / TPI
    double      seat_small_dia_mm;     // inner bore at deep end of cone
};

// Pre-computed pitch = 25.4 / TPI.  Seat small dia ≈ tube ID (≈ 0.85 × tube_od).
constexpr std::array<JicEntry, 6> kJicTable {{
    { "-04",  6.35,  11.11, 25.4 / 20.0,  5.40 },   // 7/16-20 UNF
    { "-06",  9.53,  14.29, 25.4 / 18.0,  8.10 },   // 9/16-18 UNF
    { "-08", 12.70, 19.05,  25.4 / 16.0, 10.80 },   // 3/4-16 UNF
    { "-10", 15.88, 22.23,  25.4 / 14.0, 13.50 },   // 7/8-14 UNF
    { "-12", 19.05, 26.99,  25.4 / 12.0, 16.20 },   // 1 1/16-12 UNF
    { "-16", 25.40, 33.34,  25.4 / 12.0, 21.60 },   // 1 5/16-12 UNF
}};

const JicEntry* find(const std::string& s)
{
    for (const auto& e : kJicTable) if (s == e.size) return &e;
    return nullptr;
}

}  // namespace

bool resolveJicSize(const std::string& size,
                    double& thread_major_dia_mm,
                    double& thread_pitch_mm,
                    double& seat_small_dia_mm)
{
    const JicEntry* e = find(size);
    if (!e) return false;
    thread_major_dia_mm = e->thread_major_dia_mm;
    thread_pitch_mm     = e->thread_pitch_mm;
    seat_small_dia_mm   = e->seat_small_dia_mm;
    return true;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.chamfer_leg_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "jic_flare_port_seat: chamfer_leg_mm must be >= 0");
    }
    if (in.jic_size.empty()) {
        r.add("DFM-JIC-SIZE", "error",
              "jic_flare_port_seat: jic_size is empty");
        return r;
    }
    const JicEntry* e = find(in.jic_size);
    if (!e) {
        r.add("DFM-JIC-SIZE", "error",
              "jic_flare_port_seat: unknown jic_size '" + in.jic_size +
              "' (supported: -04, -06, -08, -10, -12, -16)");
        return r;
    }

    const double threadDia    = e->thread_major_dia_mm;
    const double threadEngage = (in.thread_engagement_mm > 0.0)
        ? in.thread_engagement_mm
        : threadDia;   // default = 1 × dia

    if (threadEngage < 0.5 * threadDia) {
        r.add("DFM-JIC-DEPTH", "error",
              "jic_flare_port_seat: thread_engagement " +
              std::to_string(threadEngage) + " mm < 0.5 × thread_dia " +
              std::to_string(threadDia) + " mm (won't seal at rated PSI)");
    }

    // Seat depth = half (threadDia - seat_small_dia) / tan(37°).
    const double seatRadialDelta = (threadDia - e->seat_small_dia_mm) / 2.0;
    const double seatDepth = seatRadialDelta / std::tan(kJicHalfAngleDeg * M_PI / 180.0);
    const double totalDepth = seatDepth + threadEngage;

    if (totalDepth > in.part_thickness_mm) {
        r.add("DFM-JIC-WALL", "error",
              "jic_flare_port_seat: required depth " + std::to_string(totalDepth) +
              " mm > part_thickness " + std::to_string(in.part_thickness_mm) + " mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "jic_flare_port_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const JicEntry* e = find(in.jic_size);
    if (!e) throw SkillError("jic_flare_port_seat: jic_size lookup failed unexpectedly");

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("jic_flare_port_seat: face_id datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    constexpr double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt entryStart(
        in.position_x_mm, in.position_y_mm,
        (adir.Z() < 0) ? zMax + kOverhang : zMin - kOverhang);
    if (std::abs(adir.X()) > 1e-6 || std::abs(adir.Y()) > 1e-6) {
        // Non-axis-aligned: walk back along -adir from a center sample.
        const double bboxDiag = std::sqrt(
            (xMax - xMin) * (xMax - xMin) +
            (yMax - yMin) * (yMax - yMin) +
            (zMax - zMin) * (zMax - zMin));
        entryStart = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    }

    const double threadDia    = e->thread_major_dia_mm;
    const double threadR      = threadDia / 2.0;
    const double seatSmallR   = e->seat_small_dia_mm / 2.0;
    const double threadEngage = (in.thread_engagement_mm > 0.0)
        ? in.thread_engagement_mm : threadDia;
    const double seatRadialDelta = (threadDia - e->seat_small_dia_mm) / 2.0;
    const double seatDepth = seatRadialDelta / std::tan(kJicHalfAngleDeg * M_PI / 180.0);

    // ── Sub-cut #2 (top): UN/UNF straight thread region ─────────────────
    // We model it as a cylindrical clearance bore at threadR (the helical
    // thread itself is metadata for CAPP; the bore captures the major
    // dia for STEP round-trip).
    const gp_Ax2 threadAx(entryStart, adir);
    const TopoDS_Shape threadBore = pr::cylinder(
        threadAx, threadR, threadEngage + kOverhang);

    // ── Sub-cut #1: 37° flare seat (cone frustum) ───────────────────────
    // Starts at axial position = threadEngage along adir, big end matches
    // threadDia, small end = seat_small_dia, depth = seatDepth.
    const gp_Pnt seatStart(
        in.position_x_mm + adir.X() * threadEngage,
        in.position_y_mm + adir.Y() * threadEngage,
        entryStart.Z() + adir.Z() * threadEngage);
    const gp_Ax2 seatAx(seatStart, adir);
    const TopoDS_Shape seat = pr::coneFrustum(
        seatAx, threadR, seatSmallR, seatDepth + kOverhang);

    // ── Sub-cut #3: top sealing chamfer (45° bevel into the material) ──
    // Same geometry as threaded_npt_port: cone narrows from
    // (threadR + leg) at the face to threadR at depth `leg`.
    TopoDS_Shape chamfer;
    if (in.chamfer_leg_mm > 0.0) {
        const gp_Pnt chamfOrigin(
            in.position_x_mm,
            in.position_y_mm,
            entryStart.Z());
        const gp_Ax2 chamfAx(chamfOrigin, adir);
        chamfer = pr::coneFrustum(
            chamfAx,
            threadR + in.chamfer_leg_mm,
            threadR,
            in.chamfer_leg_mm);
    }

    // Pre-fuse the concentric cutters into one cutter.
    TopoDS_Shape unified = pr::fuse(threadBore, seat);
    if (!chamfer.IsNull()) unified = pr::fuse(unified, chamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), unified);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "jic_size",             in.jic_size },
        { "thread_engagement_mm", threadEngage },
        { "chamfer_leg_mm",       in.chamfer_leg_mm },
        { "part_thickness_mm",    in.part_thickness_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "subfeature_count",        in.chamfer_leg_mm > 0.0 ? 3 : 2 },
        { "jic_size",                in.jic_size },
        { "thread_major_dia_mm",     threadDia },                  // derived
        { "thread_pitch_mm",         e->thread_pitch_mm },          // derived
        { "tube_od_mm",              e->tube_od_mm },               // derived
        { "seat_small_dia_mm",       e->seat_small_dia_mm },        // derived
        { "seat_half_angle_deg",     kJicHalfAngleDeg },            // standard
        { "seat_depth_mm",           seatDepth },                   // derived
        { "thread_engagement_mm",    threadEngage },
        { "chamfer_leg_mm",          in.chamfer_leg_mm },
        { "axis_dir",                { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "thread_tap;flare_seat_cutter;chamfer_mill";
    tooling.tool_dia_mm       = threadDia;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 120.0;
    tooling.est_cycle_time_s  = std::max(6.0, (threadEngage + seatDepth) * 1.0);
    const double threadVol = M_PI * threadR * threadR * threadEngage;
    const double seatVol   = M_PI * seatDepth / 3.0
                           * (threadR * threadR + threadR * seatSmallR + seatSmallR * seatSmallR);
    tooling.stock_removed_mm3 = threadVol + seatVol;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", 2.0 * seatSmallR },
              { "depth_mm", threadEngage + seatDepth } },
            { { "tool_type", "thread_tap" },
              { "for", "JIC " + in.jic_size },
              { "thread_dia_mm", threadDia },
              { "pitch_mm", e->thread_pitch_mm },
              { "engagement_mm", threadEngage } },
            { { "tool_type", "flare_seat_cutter" },
              { "half_angle_deg", kJicHalfAngleDeg },
              { "depth_mm", seatDepth } },
            { { "tool_type", "chamfer_mill" },
              { "leg_mm", in.chamfer_leg_mm },
              { "angle_deg", 45.0 } },
        } },
        { "note", "37 deg JIC flare seat with UN/UNF retention threads" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::jic_flare_port_seat applied: {} thread={} engage={} seatD={}",
                  in.jic_size, threadDia, threadEngage, seatDepth);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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
    return out;
}

}  // namespace koocadcam::skill::jic_flare_port_seat
