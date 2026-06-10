// @lat: [[engine/skills#adjuster_screw_pocket_compound]]

#include "adjuster_screw_pocket_compound.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::adjuster_screw_pocket_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Local non-overlap tables ──────────────────────────────────────────────
//
// The base M-size nominal data (M3, M4, M5, M6, M8) lives in the central
// _iso_thread_table.hpp.  Two columns here are NOT in the central table and
// remain local:
//   • fine-pitch ISO 261 / ISO 68-1 pilot & pitch  (central table is coarse)
//   • DIN 934 hex-nut across-flats                  (not modelled centrally)
namespace {

namespace tt = koocadcam::skill::thread_table;

struct FineEntry {
    const char* size;          // fine designator e.g. "M6x0.75"
    double      pilot_dia_mm;  // ISO 965 60% engagement pilot for fine pitch
    double      pitch_mm;      // fine pitch
};

constexpr std::array<FineEntry, 5> kFineTable {{
    { "M3x0.35",  2.65, 0.35 },
    { "M4x0.5",   3.50, 0.50 },
    { "M5x0.5",   4.50, 0.50 },
    { "M6x0.75",  5.25, 0.75 },
    { "M8x1",     7.00, 1.00 },
}};

// DIN 934 hex-nut across-flats (column not in central table).
struct LockNutAF {
    const char* M;
    double      across_flats_mm;
};
constexpr std::array<LockNutAF, 5> kLockNutAF {{
    { "M3", 5.5  },
    { "M4", 7.0  },
    { "M5", 8.0  },
    { "M6", 10.0 },
    { "M8", 13.0 },
}};

const FineEntry* findFine(const std::string& sz) {
    for (const auto& e : kFineTable)
        if (sz == e.size) return &e;
    return nullptr;
}

}  // namespace

double finePilotDiameterFor(const std::string& fine_thread) {
    const auto* e = findFine(fine_thread);
    return e ? e->pilot_dia_mm : 0.0;
}
double finePitchFor(const std::string& fine_thread) {
    const auto* e = findFine(fine_thread);
    return e ? e->pitch_mm : 0.0;
}
double lockNutAcrossFlatsFor(const std::string& lock_nut_M) {
    // Only proceed if the M-size is a known central-table entry (validates
    // the key against ISO 261); then look up the DIN 934 AF locally.
    if (!tt::findMetric(lock_nut_M)) return 0.0;
    for (const auto& e : kLockNutAF)
        if (lock_nut_M == e.M) return e.across_flats_mm;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.fine_thread.empty() || finePilotDiameterFor(in.fine_thread) <= 0.0) {
        r.add("DFM-ADJ-FINE", "error",
              "adjuster_screw_pocket_compound: unknown fine_thread '" +
              in.fine_thread +
              "' (supported: M3x0.35/M4x0.5/M5x0.5/M6x0.75/M8x1)");
        return r;
    }
    if (in.lock_nut_M.empty() || lockNutAcrossFlatsFor(in.lock_nut_M) <= 0.0) {
        r.add("DFM-ADJ-NUT", "error",
              "adjuster_screw_pocket_compound: unknown lock_nut_M '" +
              in.lock_nut_M + "' (supported: M3/M4/M5/M6/M8)");
        return r;
    }

    const double pilotDia = finePilotDiameterFor(in.fine_thread);
    const double pitch    = finePitchFor(in.fine_thread);
    const double nutAF    = lockNutAcrossFlatsFor(in.lock_nut_M);

    if (in.tap_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "adjuster_screw_pocket_compound: tap_depth_mm must be > 0");
    }
    if (in.tap_depth_mm > 0.0 && pitch > 0.0 && in.tap_depth_mm < 3.0 * pitch) {
        r.add("DFM-ADJ-THREAD-ENGAGE", "error",
              "adjuster_screw_pocket_compound: tap_depth " +
              std::to_string(in.tap_depth_mm) +
              " mm < 3 × pitch (" + std::to_string(3.0 * pitch) +
              " mm) — thread engagement insufficient");
    }
    if (in.nut_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "adjuster_screw_pocket_compound: nut_thickness_mm must be > 0");
    }
    if (nutAF + 0.4 <= pilotDia) {
        r.add("DFM-ADJ-GEOM", "error",
              "adjuster_screw_pocket_compound: lock-nut seat ("
              + std::to_string(nutAF + 0.4) +
              " mm) must be > pilot dia ("
              + std::to_string(pilotDia) + " mm)");
    }
    if (in.scribe_groove_width_mm < 0.2) {
        r.add("DFM-002", "error",
              "adjuster_screw_pocket_compound: scribe groove width "
              + std::to_string(in.scribe_groove_width_mm) +
              " mm < 0.2 mm (min CNC engraver kerf)");
    }
    if (in.scribe_groove_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "adjuster_screw_pocket_compound: scribe groove depth must be > 0");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "adjuster_screw_pocket_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(
        "adjuster_screw_pocket_compound: entry_face datum unresolved");

    const double pilotDia = finePilotDiameterFor(in.fine_thread);
    const double pitch    = finePitchFor(in.fine_thread);
    const double nutAF    = lockNutAcrossFlatsFor(in.lock_nut_M);

    // Lock-nut seat: across-flats + 0.4 mm clearance for hex tool engagement.
    const double seatDia   = nutAF + 0.4;
    const double seatDepth = in.nut_thickness_mm + 0.2;

    // Scribe groove: annular ring on the rim, outer dia = seatDia + 1.5 mm,
    // inner dia = seatDia + 1.5 - scribe_groove_width × 2 = seatDia + 1.5 -
    // 2 × scribe_groove_width.  Sat slightly above the entry plane is
    // useless for an annular ring; instead we cut a narrow ring INTO the
    // top face concentric to the seat at the rim radius.
    const double scribeOuterR = seatDia / 2.0 + 1.0 + in.scribe_groove_width_mm;
    const double scribeInnerR = seatDia / 2.0 + 1.0;
    const double scribeDepth  = in.scribe_groove_depth_mm;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt entryPnt;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0)
            entryPnt = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        else
            entryPnt = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
    } else {
        entryPnt = gp_Pnt(in.position_x_mm, in.position_y_mm, (zMin + zMax) / 2.0);
    }

    const gp_Ax2 toolAx(entryPnt, adir);

    // Sub-feature 1 — pilot (fine-pitch tapped) hole.
    const TopoDS_Shape pilotCutter =
        pr::cylinder(toolAx, pilotDia / 2.0, in.tap_depth_mm + kOverhang);

    // Sub-feature 2 — lock-nut counterbore seat.
    const TopoDS_Shape seatCutter =
        pr::cylinder(toolAx, seatDia / 2.0, seatDepth + kOverhang);

    // Sub-feature 3 — scribe groove on rim (annular ring tool).
    const TopoDS_Shape scribeCutter =
        pr::annularRing(toolAx, scribeOuterR, scribeInnerR, scribeDepth + kOverhang);

    // Pilot and seat are concentric overlapping cylinders → fuse first.
    const TopoDS_Shape fused1     = pr::fuse(pilotCutter, seatCutter);
    const TopoDS_Shape fusedAll   = pr::fuse(fused1, scribeCutter);
    const TopoDS_Shape newShape   = pr::cut(wp.shape(), fusedAll);

    json params = {
        { "entry_face_id",          *entryId },
        { "position_x_mm",          in.position_x_mm },
        { "position_y_mm",          in.position_y_mm },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
        { "fine_thread",            in.fine_thread },
        { "lock_nut_M",             in.lock_nut_M },
        { "tap_depth_mm",           in.tap_depth_mm },
        { "nut_thickness_mm",       in.nut_thickness_mm },
        { "scribe_groove_width_mm", in.scribe_groove_width_mm },
        { "scribe_groove_depth_mm", in.scribe_groove_depth_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  3 },
        { "fine_thread",       in.fine_thread },
        { "lock_nut_M",        in.lock_nut_M },
        { "pilot_dia_mm",      pilotDia },
        { "pitch_mm",          pitch },
        { "lock_nut_af_mm",    nutAF },
        { "seat_dia_mm",       seatDia },
        { "seat_depth_mm",     seatDepth },
        { "tap_depth_mm",      in.tap_depth_mm },
        { "scribe_outer_r_mm", scribeOuterR },
        { "scribe_inner_r_mm", scribeInnerR },
        { "scribe_depth_mm",   scribeDepth },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill;tap;counterbore;engraver";
    tooling.tool_dia_mm      = seatDia;
    tooling.tool_length_mm   = in.tap_depth_mm * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double pilotR = pilotDia / 2.0;
    const double seatR  = seatDia  / 2.0;
    tooling.stock_removed_mm3 =
        M_PI * pilotR * pilotR * in.tap_depth_mm +
        M_PI * (seatR * seatR - pilotR * pilotR) * seatDepth +
        M_PI * (scribeOuterR * scribeOuterR - scribeInnerR * scribeInnerR)
             * scribeDepth;
    tooling.est_cycle_time_s = std::max(5.0,
        in.tap_depth_mm * 0.2 + seatDepth * 0.3 + 2.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", pilotDia },
              { "depth_mm",    in.tap_depth_mm } },
            { { "tool_type", "tap" },
              { "thread_size", in.fine_thread },
              { "pitch_mm",    pitch },
              { "tap_depth_mm", in.tap_depth_mm } },
            { { "tool_type", "counterbore" },
              { "tool_dia_mm", seatDia },
              { "depth_mm",    seatDepth },
              { "purpose",     "lock-nut seat (DIN 934)" } },
            { { "tool_type", "engraver" },
              { "tool_dia_mm", in.scribe_groove_width_mm },
              { "depth_mm",    scribeDepth },
              { "purpose",     "indicator scribe groove on rim" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::adjuster_screw_pocket_compound applied: "
                  "thread={} nut={} pilot={} seat={} scribe={}",
                  in.fine_thread, in.lock_nut_M, pilotDia, seatDia,
                  in.scribe_groove_width_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition — metadata replay ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "via", "metadata_replay" },
              { "subfeature_count", 3 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::adjuster_screw_pocket_compound
