// @lat: [[engine/reverse-route#corpus-cli]]
//
// koo_corpus — breakthrough plan B7.1–B7.2 MVP: self-labelled synthetic
// corpus generator + recall meter for the precise recognizer tier.
//
// For each precise-tier skill with an apply(), this tool:
//   1. synthesizes `samples_per_skill` single-feature parts on a fixed
//      80x80x15 cuboid stock; parameters are drawn from DFM-valid ranges
//      with a DETERMINISTIC std::mt19937 (seed = skill_index*1000 + sample);
//   2. STEP-round-trips the part through a temp file so all feature metadata
//      is stripped (features() must be 0 afterwards — otherwise the sample
//      is recorded as a metadata_leak and never counts as recognized);
//   3. runs the official RE pipeline re::dedupe(re::analyze(wp));
//   4. scores a match: same skill_id, confidence >= 0.7, every key dimension
//      within 5 % relative OR 0.2 mm absolute, and (for positional skills)
//      entry-position error <= 0.5 mm.  fillet/chamfer carry no position —
//      dimensions only.
//
// Each JSONL row also records `matched_pre_dedupe` (same criteria evaluated
// against the raw analyze() output, before dedupe) so a recall gap can be
// localized: "the recognizer never fires" vs "it fires but is shadowed by
// a higher-confidence competitor in dedupe (e.g. drill_hole at 0.95)".
//
// Output: stdout = one JSONL row per sample; stderr = per-skill summary.
// Exit:   without --gate → always 0 (measurement mode);
//         with --gate r  → 0 iff every skill's recall >= r, else 1.
//
//   Usage: koo_corpus [samples_per_skill=3] [--gate <recall>] [--scale <f>]
//     --scale f multiplies every length parameter and the stock by f
//     (scale-gap measurement).  Parameters whose DFM windows are ABSOLUTE
//     (drill dia >= 0.8, ream enlarge [0.02,0.30], spot_face dia/depth,
//     fillet radius >= 0.2, chamfer size >= 0.1) are clamped back into
//     their windows after scaling so every generated input stays DFM-valid.
//     bolt_hole_metric_spec holes are ISO-273 table sizes and never scale.

#include "io/StepIO.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"

#include "skills/bolt_hole_metric_spec.hpp"
#include "skills/bore_cylindrical.hpp"
#include "skills/bore_with_shelf.hpp"
#include "skills/chamfer_edge.hpp"
#include "skills/counterbore.hpp"
#include "skills/countersink.hpp"
#include "skills/drill_hole.hpp"
#include "skills/drill_through_hole.hpp"
#include "skills/fillet_edge.hpp"
#include "skills/mill_circular_pocket.hpp"
#include "skills/mill_rect_pocket.hpp"
#include "skills/mill_slot.hpp"
#include "skills/ream.hpp"
#include "skills/spot_drill.hpp"
#include "skills/spot_face.hpp"

#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace koocadcam;
using json = nlohmann::json;

namespace {

// ── Matching thresholds (spec, B7.2) ──────────────────────────────────────
constexpr double kConfThreshold = 0.7;   // re pipeline's own RE threshold
constexpr double kDimAbsTol     = 0.2;   // mm — absolute dimension tolerance
constexpr double kDimRelTol     = 0.05;  // 5 % — relative dimension tolerance
constexpr double kPosTol        = 0.5;   // mm — entry-position tolerance

const std::vector<const char*> kNoAliases;   // strict-id evaluation

// ── Phase-1 stock (scaled by --scale) ─────────────────────────────────────
constexpr double kStockW = 80.0;
constexpr double kStockH = 80.0;
constexpr double kStockT = 15.0;

constexpr double kInf = std::numeric_limits<double>::infinity();

// ── Deterministic sampling helpers ─────────────────────────────────────────
double U(std::mt19937& rng, double lo, double hi)
{
    return std::uniform_real_distribution<double>(lo, hi)(rng);
}

int pickIndex(std::mt19937& rng, int n)
{
    return std::uniform_int_distribution<int>(0, n - 1)(rng);
}

std::shared_ptr<skill::Workpiece> makeStock(double f)
{
    return skill::createCuboidStock(kStockW * f, kStockH * f, kStockT * f);
}

// ── Position matchers (recovered_params → error in mm) ────────────────────
using PosFn = std::function<double(const json&)>;

PosFn posAtKeys(const char* kx, const char* ky, double ex, double ey)
{
    return [kx, ky, ex, ey](const json& rp) -> double {
        if (!rp.is_object() || !rp.contains(kx) || !rp.contains(ky) ||
            !rp[kx].is_number() || !rp[ky].is_number())
            return kInf;
        const double dx = rp[kx].get<double>() - ex;
        const double dy = rp[ky].get<double>() - ey;
        return std::sqrt(dx * dx + dy * dy);
    };
}

// ream's recognize() reports the cylinder axis base as "axis_location"=[x,y,z].
PosFn posAtAxisLocation(double ex, double ey)
{
    return [ex, ey](const json& rp) -> double {
        if (!rp.is_object() || !rp.contains("axis_location")) return kInf;
        const json& a = rp["axis_location"];
        if (!a.is_array() || a.size() < 2 || !a[0].is_number() || !a[1].is_number())
            return kInf;
        const double dx = a[0].get<double>() - ex;
        const double dy = a[1].get<double>() - ey;
        return std::sqrt(dx * dx + dy * dy);
    };
}

// mill_slot's start/end may come back swapped — take the better ordering.
PosFn posSlotEndpoints(double sx, double sy, double ex, double ey)
{
    return [sx, sy, ex, ey](const json& rp) -> double {
        static const char* kKeys[] = { "start_x_mm", "start_y_mm",
                                       "end_x_mm",   "end_y_mm" };
        for (const char* k : kKeys)
            if (!rp.is_object() || !rp.contains(k) || !rp[k].is_number())
                return kInf;
        const double ax = rp["start_x_mm"].get<double>();
        const double ay = rp["start_y_mm"].get<double>();
        const double bx = rp["end_x_mm"].get<double>();
        const double by = rp["end_y_mm"].get<double>();
        auto d = [](double x0, double y0, double x1, double y1) {
            return std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
        };
        const double fwd  = std::max(d(ax, ay, sx, sy), d(bx, by, ex, ey));
        const double swap = std::max(d(ax, ay, ex, ey), d(bx, by, sx, sy));
        return std::min(fwd, swap);
    };
}

// ── One synthesized sample ─────────────────────────────────────────────────
struct Sample
{
    json                                            params;  // sampled inputs
    TopoDS_Shape                                    shape;   // final geometry
    // recovered_params key → expected value (mm); every entry must pass.
    std::vector<std::pair<std::string, double>>     dims;
    // empty function ⇔ skill has no position (fillet/chamfer).
    PosFn                                           posErr;
};

using Generator = std::function<Sample(std::mt19937&, double)>;

struct SkillCase
{
    const char* id;
    Generator   gen;
};

// Geometric-equivalence aliases: ids accepted as a CORRECT post-dedupe answer
// for a sample of the keyed skill.  drill / bore_cylindrical /
// mill_circular_pocket leave the SAME two faces (wall + entry/bottom) on
// these synthetic single-feature parts, so dedupe legitimately resolves them
// by confidence — recovering the alias with the right dimensions is correct
// understanding, not a recall miss.  (bolt_hole_metric_spec has no usable
// alias: its dim key clearance_dia_mm does not exist on drill_hole's
// recovered params — it stays gate-exempt until its recognize() emits an
// entry_face_id and can win the specificity rule itself.)
const std::vector<const char*>& aliasesFor(const std::string& id)
{
    static const std::vector<const char*> kNone;
    static const std::vector<const char*> kDrill   = { "drill_hole" };
    static const std::vector<const char*> kBoreEq  = { "drill_hole",
                                                       "mill_circular_pocket" };
    static const std::vector<const char*> kPockEq  = { "drill_hole",
                                                       "bore_cylindrical" };
    if (id == "drill_through_hole")   return kDrill;
    if (id == "bore_cylindrical")     return kBoreEq;
    if (id == "mill_circular_pocket") return kPockEq;
    return kNone;
}

// ── The 15 precise-tier generators (parameter ranges stay DFM-valid) ──────
const std::vector<SkillCase>& corpusCases()
{
    static const std::vector<SkillCase> kCases = {

        { "drill_hole", [](std::mt19937& rng, double f) -> Sample {
            const double cx    = U(rng, 16.0, 64.0) * f;
            const double cy    = U(rng, 16.0, 64.0) * f;
            const double dia   = std::max(0.8, U(rng, 2.0, 12.0) * f);  // DFM-002
            const double depth = U(rng, 2.0, 8.0) * f;
            skill::drill_hole::Input in;
            in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm = cx;  in.position_y_mm = cy;
            in.axis_dir      = gp_Dir(0, 0, -1);
            in.diameter_mm   = dia; in.depth_mm = depth;
            Sample s;
            s.shape  = skill::drill_hole::apply(*makeStock(f), in).workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "diameter_mm", dia }, { "depth_mm", depth } };
            s.dims   = { { "diameter_mm", dia }, { "depth_mm", depth } };
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },

        { "drill_through_hole", [](std::mt19937& rng, double f) -> Sample {
            const double cx  = U(rng, 16.0, 64.0) * f;
            const double cy  = U(rng, 16.0, 64.0) * f;
            const double dia = std::max(0.8, U(rng, 2.0, 12.0) * f);    // DFM-002
            skill::drill_through_hole::Input in;
            in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm = cx;  in.position_y_mm = cy;
            in.axis_dir      = gp_Dir(0, 0, -1);
            in.diameter_mm   = dia;
            Sample s;
            s.shape  = skill::drill_through_hole::apply(*makeStock(f), in)
                           .workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "diameter_mm", dia }, { "through_hole", true } };
            s.dims   = { { "diameter_mm", dia } };   // no depth on a through hole
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },

        { "counterbore", [](std::mt19937& rng, double f) -> Sample {
            const double cx         = U(rng, 16.0, 64.0) * f;
            const double cy         = U(rng, 16.0, 64.0) * f;
            const double pilotDia   = std::max(0.8, U(rng, 2.0, 8.0) * f);  // DFM-002
            const double seatDia    = pilotDia + U(rng, 2.0, 5.0) * f;      // > pilot
            const double seatDepth  = U(rng, 1.5, 4.0) * f;
            const double pilotDepth = seatDepth + U(rng, 3.0, 8.0) * f;     // > seat
            skill::counterbore::Input in;
            in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm  = cx;        in.position_y_mm = cy;
            in.axis_dir       = gp_Dir(0, 0, -1);
            in.pilot_dia_mm   = pilotDia;  in.pilot_depth_mm = pilotDepth;
            in.seat_dia_mm    = seatDia;   in.seat_depth_mm  = seatDepth;
            Sample s;
            s.shape  = skill::counterbore::apply(*makeStock(f), in).workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "pilot_dia_mm", pilotDia }, { "pilot_depth_mm", pilotDepth },
                         { "seat_dia_mm", seatDia }, { "seat_depth_mm", seatDepth } };
            s.dims   = { { "pilot_dia_mm", pilotDia }, { "seat_dia_mm", seatDia },
                         { "pilot_depth_mm", pilotDepth }, { "seat_depth_mm", seatDepth } };
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },

        { "countersink", [](std::mt19937& rng, double f) -> Sample {
            static const double kAngles[] = { 82.0, 90.0, 100.0, 120.0 };
            const double cx       = U(rng, 16.0, 64.0) * f;
            const double cy       = U(rng, 16.0, 64.0) * f;
            const double pilotDia = std::max(0.8, U(rng, 2.0, 8.0) * f);    // DFM-002
            const double coneTop  = pilotDia + U(rng, 1.5, 5.0) * f;        // > pilot
            const double angle    = kAngles[pickIndex(rng, 4)];
            const double extra    = U(rng, 2.5, 6.0) * f;
            skill::countersink::Input in;
            in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm   = cx;       in.position_y_mm = cy;
            in.axis_dir        = gp_Dir(0, 0, -1);
            in.pilot_dia_mm    = pilotDia; in.cone_top_dia_mm = coneTop;
            in.cone_angle_deg  = angle;
            // pilot_depth is measured from the ENTRY face — it must exceed
            // the derived cone depth so a real cylindrical pilot remains.
            in.pilot_depth_mm  = skill::countersink::computeConeDepth(in) + extra;
            Sample s;
            s.shape  = skill::countersink::apply(*makeStock(f), in).workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "pilot_dia_mm", pilotDia },
                         { "pilot_depth_mm", in.pilot_depth_mm },
                         { "cone_top_dia_mm", coneTop }, { "cone_angle_deg", angle } };
            s.dims   = { { "pilot_dia_mm", pilotDia }, { "cone_top_dia_mm", coneTop },
                         { "pilot_depth_mm", in.pilot_depth_mm } };
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },

        { "chamfer_edge", [](std::mt19937& rng, double f) -> Sample {
            // Top-rim Z-band (4 edges → recognizer cluster, conf 0.85).
            // angle is fixed at 45°: phase-1 apply() is symmetric and the
            // recognizer always reports 45°.
            const double size = std::max(0.1, U(rng, 0.3, 1.5) * f);   // >= 0.1 abs
            const double topZ = kStockT * f;
            skill::chamfer_edge::Input in;
            in.edge_selector   = skill::fillet_edge::EdgesAtZBand{ topZ, 1e-3 };
            in.chamfer_size_mm = size;
            in.angle_deg       = 45.0;
            Sample s;
            s.shape  = skill::chamfer_edge::apply(*makeStock(f), in).workpiece->shape();
            s.params = { { "edges_at_z_mm", topZ }, { "chamfer_size_mm", size },
                         { "angle_deg", 45.0 } };
            s.dims   = { { "chamfer_size_mm", size } };
            // no position — edge skill
            return s;
        } },

        { "fillet_edge", [](std::mt19937& rng, double f) -> Sample {
            const double radius = std::max(0.2, U(rng, 0.3, 2.0) * f);  // DFM-004
            const double topZ   = kStockT * f;
            skill::fillet_edge::Input in;
            in.edge_selector = skill::fillet_edge::EdgesAtZBand{ topZ, 1e-3 };
            in.radius_mm     = radius;
            Sample s;
            s.shape  = skill::fillet_edge::apply(*makeStock(f), in).workpiece->shape();
            s.params = { { "edges_at_z_mm", topZ }, { "radius_mm", radius } };
            s.dims   = { { "radius_mm", radius } };
            // no position — edge skill
            return s;
        } },

        { "bore_cylindrical", [](std::mt19937& rng, double f) -> Sample {
            const double cx    = U(rng, 16.0, 64.0) * f;
            const double cy    = U(rng, 16.0, 64.0) * f;
            const double dia   = std::max(0.8, U(rng, 6.0, 12.0) * f);  // DFM-002
            const double depth = U(rng, 2.0, 8.0) * f;                  // ratio <= 4
            skill::bore_cylindrical::Input in;
            in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm = cx;  in.position_y_mm = cy;
            in.axis_dir      = gp_Dir(0, 0, -1);
            in.diameter_mm   = dia; in.depth_mm = depth;
            in.tolerance_class = "H7";
            Sample s;
            s.shape  = skill::bore_cylindrical::apply(*makeStock(f), in)
                           .workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "diameter_mm", dia }, { "depth_mm", depth },
                         { "tolerance_class", "H7" } };
            s.dims   = { { "diameter_mm", dia }, { "depth_mm", depth } };
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },

        { "bore_with_shelf", [](std::mt19937& rng, double f) -> Sample {
            const double cx         = U(rng, 16.0, 64.0) * f;
            const double cy         = U(rng, 16.0, 64.0) * f;
            const double upperDia   = std::max(0.8, U(rng, 3.0, 6.0) * f);  // DFM-002
            const double lowerDia   = upperDia + U(rng, 2.0, 5.0) * f;      // > upper
            const double upperDepth = U(rng, 2.0, 4.0) * f;
            const double lowerDepth = U(rng, 2.0, 5.0) * f;   // from shelf; total < 9f
            skill::bore_with_shelf::Input in;
            in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm  = cx;        in.position_y_mm = cy;
            in.axis_dir       = gp_Dir(0, 0, -1);
            in.upper_dia_mm   = upperDia;  in.upper_depth_mm = upperDepth;
            in.lower_dia_mm   = lowerDia;  in.lower_depth_mm = lowerDepth;
            Sample s;
            s.shape  = skill::bore_with_shelf::apply(*makeStock(f), in)
                           .workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "upper_dia_mm", upperDia }, { "upper_depth_mm", upperDepth },
                         { "lower_dia_mm", lowerDia }, { "lower_depth_mm", lowerDepth } };
            s.dims   = { { "upper_dia_mm", upperDia }, { "lower_dia_mm", lowerDia },
                         { "upper_depth_mm", upperDepth }, { "lower_depth_mm", lowerDepth } };
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },

        { "ream", [](std::mt19937& rng, double f) -> Sample {
            const double cx    = U(rng, 16.0, 64.0) * f;
            const double cy    = U(rng, 16.0, 64.0) * f;
            const double dia   = std::max(0.8, U(rng, 4.0, 8.0) * f);
            const double depth = U(rng, 6.0, 10.0) * f;
            // enlarge_by has an ABSOLUTE DFM window [0.02, 0.30] mm.
            const double enlarge = std::clamp(U(rng, 0.05, 0.20) * f, 0.02, 0.30);

            skill::drill_hole::Input din;
            din.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            din.position_x_mm = cx;  din.position_y_mm = cy;
            din.axis_dir      = gp_Dir(0, 0, -1);
            din.diameter_mm   = dia; din.depth_mm = depth;
            auto drilled = skill::drill_hole::apply(*makeStock(f), din).workpiece;

            skill::ream::Input in;
            in.existing_hole_datum = skill::FaceCylinderByAxis{
                gp_Ax1(gp_Pnt(cx, cy, kStockT * f), gp_Dir(0, 0, -1)), 5.0 };
            in.enlarge_by_mm = enlarge;
            Sample s;
            s.shape  = skill::ream::apply(*drilled, in).workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "base_drill_dia_mm", dia }, { "base_drill_depth_mm", depth },
                         { "enlarge_by_mm", enlarge },
                         { "expected_final_dia_mm", dia + 2.0 * enlarge } };
            s.dims   = { { "new_radius_mm", dia / 2.0 + enlarge } };
            s.posErr = posAtAxisLocation(cx, cy);
            return s;
        } },

        { "spot_drill", [](std::mt19937& rng, double f) -> Sample {
            static const double kAngles[] = { 90.0, 120.0 };
            const double cx    = U(rng, 16.0, 64.0) * f;
            const double cy    = U(rng, 16.0, 64.0) * f;
            const double dia   = std::max(0.8, U(rng, 2.0, 8.0) * f);   // DFM-002
            const double angle = kAngles[pickIndex(rng, 2)];
            skill::spot_drill::Input in;
            in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm  = cx;  in.position_y_mm = cy;
            in.axis_dir       = gp_Dir(0, 0, -1);
            in.diameter_mm    = dia;
            in.cone_angle_deg = angle;
            const double expectedDepth = skill::spot_drill::computeDepth(in);
            Sample s;
            s.shape  = skill::spot_drill::apply(*makeStock(f), in).workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "diameter_mm", dia }, { "cone_angle_deg", angle },
                         { "derived_depth_mm", expectedDepth } };
            s.dims   = { { "diameter_mm", dia }, { "depth_mm", expectedDepth } };
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },

        { "spot_face", [](std::mt19937& rng, double f) -> Sample {
            const double cx        = U(rng, 16.0, 64.0) * f;
            const double cy        = U(rng, 16.0, 64.0) * f;
            const double diaDraw   = U(rng, 4.0, 12.0);
            // keep depth/dia < 0.5 so the recognizer's shallow gate holds.
            const double depthDraw = U(rng, 0.5, std::min(2.0, 0.45 * diaDraw));
            // DFM-002 (>= 3 mm cutter) and DFM-SPOTFACE-DEPTH ([0.3, 5.0])
            // are ABSOLUTE windows — clamp after scaling.
            const double dia   = std::max(3.0, diaDraw * f);
            const double depth = std::clamp(depthDraw * f, 0.3, 5.0);
            skill::spot_face::Input in;
            in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm = cx;  in.position_y_mm = cy;
            in.axis_dir      = gp_Dir(0, 0, -1);
            in.dia_mm        = dia; in.depth_mm = depth;
            Sample s;
            s.shape  = skill::spot_face::apply(*makeStock(f), in).workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "dia_mm", dia }, { "depth_mm", depth } };
            s.dims   = { { "dia_mm", dia }, { "depth_mm", depth } };
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },

        { "mill_circular_pocket", [](std::mt19937& rng, double f) -> Sample {
            const double cx    = U(rng, 16.0, 64.0) * f;
            const double cy    = U(rng, 16.0, 64.0) * f;
            const double dia   = std::max(0.8, U(rng, 6.0, 12.0) * f);  // DFM-002
            const double depth = U(rng, 2.0, 8.0) * f;                  // ratio < 2
            skill::mill_circular_pocket::Input in;
            in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm      = cx;  in.position_y_mm = cy;
            in.axis_dir           = gp_Dir(0, 0, -1);
            in.diameter_mm        = dia; in.depth_mm = depth;
            in.bottom_corner_r_mm = 0.0;                                // sharp corner
            Sample s;
            s.shape  = skill::mill_circular_pocket::apply(*makeStock(f), in)
                           .workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "diameter_mm", dia }, { "depth_mm", depth },
                         { "bottom_corner_r_mm", 0.0 } };
            s.dims   = { { "diameter_mm", dia }, { "depth_mm", depth } };
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },

        { "mill_rect_pocket", [](std::mt19937& rng, double f) -> Sample {
            const double cx      = U(rng, 16.0, 64.0) * f;
            const double cy      = U(rng, 16.0, 64.0) * f;
            const double length  = U(rng, 8.0, 16.0) * f;
            const double width   = U(rng, 6.0, 12.0) * f;
            const double depth   = U(rng, 2.0, 5.0) * f;                // <= DFM-006
            const double cornerR = std::max(0.2, U(rng, 1.0, 2.0) * f); // DFM-004
            skill::mill_rect_pocket::Input in;
            in.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.center_x_mm = cx;     in.center_y_mm = cy;
            in.axis_dir    = gp_Dir(0, 0, -1);
            in.length_mm   = length; in.width_mm = width;
            in.depth_mm    = depth;  in.corner_r_mm = cornerR;
            Sample s;
            s.shape  = skill::mill_rect_pocket::apply(*makeStock(f), in)
                           .workpiece->shape();
            s.params = { { "center_x_mm", cx }, { "center_y_mm", cy },
                         { "length_mm", length }, { "width_mm", width },
                         { "depth_mm", depth }, { "corner_r_mm", cornerR } };
            s.dims   = { { "length_mm", length }, { "width_mm", width },
                         { "depth_mm", depth }, { "corner_r_mm", cornerR } };
            s.posErr = posAtKeys("center_x_mm", "center_y_mm", cx, cy);
            return s;
        } },

        { "mill_slot", [](std::mt19937& rng, double f) -> Sample {
            const double sx    = U(rng, 16.0, 36.0) * f;
            const double sy    = U(rng, 16.0, 64.0) * f;
            const double len   = U(rng, 10.0, 25.0) * f;   // X-aligned, ex <= 61f
            const double width = std::max(0.8, U(rng, 3.0, 8.0) * f);   // DFM-002
            const double depth = U(rng, 2.0, 6.0) * f;
            const double ex = sx + len, ey = sy;
            skill::mill_slot::Input in;
            in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.start_x_mm = sx;    in.start_y_mm = sy;
            in.end_x_mm   = ex;    in.end_y_mm   = ey;
            in.axis_dir   = gp_Dir(0, 0, -1);
            in.width_mm   = width; in.depth_mm = depth;
            Sample s;
            s.shape  = skill::mill_slot::apply(*makeStock(f), in).workpiece->shape();
            s.params = { { "start_x_mm", sx }, { "start_y_mm", sy },
                         { "end_x_mm", ex }, { "end_y_mm", ey },
                         { "width_mm", width }, { "depth_mm", depth } };
            s.dims   = { { "width_mm", width }, { "depth_mm", depth } };
            s.posErr = posSlotEndpoints(sx, sy, ex, ey);
            return s;
        } },

        { "bolt_hole_metric_spec", [](std::mt19937& rng, double f) -> Sample {
            // Spec-table skill: the clearance hole is an ISO-273 size and
            // does NOT scale; only the stock (and chamfer leg) scale.
            static const char* kSizes[] = { "M3", "M4", "M5", "M6", "M8", "M10" };
            static const char* kFits[]  = { "close", "medium", "free" };
            const double cx = U(rng, 16.0, 64.0) * f;
            const double cy = U(rng, 16.0, 64.0) * f;
            const std::string size = kSizes[pickIndex(rng, 6)];
            const std::string fit  = kFits[pickIndex(rng, 3)];
            const double chamfer   = std::max(0.1, 0.5 * f);
            const double expectDia =
                skill::bolt_hole_metric_spec::clearanceDiameterFor(size, fit);
            skill::bolt_hole_metric_spec::Input in;
            in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm     = cx;   in.position_y_mm = cy;
            in.axis_dir          = gp_Dir(0, 0, -1);
            in.thread_size       = size; in.fit_class = fit;
            in.chamfer_size_mm   = chamfer;
            in.chamfer_angle_deg = 30.0;
            Sample s;
            s.shape  = skill::bolt_hole_metric_spec::apply(*makeStock(f), in)
                           .workpiece->shape();
            s.params = { { "position_x_mm", cx }, { "position_y_mm", cy },
                         { "thread_size", size }, { "fit_class", fit },
                         { "chamfer_size_mm", chamfer },
                         { "expected_clearance_dia_mm", expectDia } };
            s.dims   = { { "clearance_dia_mm", expectDia } };
            s.posErr = posAtKeys("position_x_mm", "position_y_mm", cx, cy);
            return s;
        } },
    };
    return kCases;
}

// ── Candidate-list evaluation against one sample's expectations ───────────
struct Eval
{
    bool   found   = false;   // a same-skill candidate exists in the list
    bool   matched = false;   // full match (conf + dims + position)
    double conf    = 0.0;     // best same-skill candidate's confidence
    double dimErr  = -1.0;    // its max key-dimension error (mm; may be inf)
    double posErr  = -1.0;    // its position error (mm); -1 ⇔ skill has none
};

Eval evaluate(const Sample& s, const std::string& skillId,
              const std::vector<const char*>& aliases,
              const std::vector<skill::RecognizedFeature>& cands)
{
    Eval best;
    for (const auto& c : cands) {
        bool idOk = (c.skill_id == skillId);
        for (const char* a : aliases) {
            if (idOk) break;
            idOk = (c.skill_id == a);
        }
        if (!idOk) continue;
        const json& rp = c.recovered_params;

        double maxErr = 0.0;
        bool   dimsOk = true;
        for (const auto& kv : s.dims) {
            double err = kInf;
            if (rp.is_object() && rp.contains(kv.first) && rp[kv.first].is_number())
                err = std::abs(rp[kv.first].get<double>() - kv.second);
            maxErr = std::max(maxErr, err);
            const double rel = err / std::max(std::abs(kv.second), 1e-9);
            if (err > kDimAbsTol && rel > kDimRelTol) dimsOk = false;
        }

        double pe    = -1.0;
        bool   posOk = true;
        if (s.posErr) {
            pe    = s.posErr(rp);
            posOk = pe <= kPosTol;
        }

        const bool m = (c.confidence >= kConfThreshold) && dimsOk && posOk;
        const double bestErr =
            (best.found && best.dimErr >= 0.0) ? best.dimErr : kInf;
        const bool better =
            !best.found ||
            (m && !best.matched) ||
            (m == best.matched && maxErr < bestErr);
        if (better) {
            best.found   = true;
            best.matched = m;
            best.conf    = c.confidence;
            best.dimErr  = maxErr;
            best.posErr  = pe;
        }
    }
    return best;
}

// ── Per-skill aggregation ──────────────────────────────────────────────────
struct Agg
{
    int    n = 0, hits = 0, hitsPre = 0, leaks = 0, errors = 0;
    double errSum = 0.0, errMax = 0.0;
    int    errN = 0;
};

}  // namespace

int main(int argc, char* argv[])
{
    int    samplesPerSkill = 3;
    double gate            = -1.0;
    bool   hasGate         = false;
    double gatePre         = -1.0;
    bool   hasGatePre      = false;
    double scaleF          = 1.0;
    std::set<std::string> exempt;

    for (int a = 1; a < argc; ++a) {
        const std::string arg = argv[a];
        if (arg == "--gate" && a + 1 < argc) {
            gate    = std::atof(argv[++a]);
            hasGate = true;
        } else if (arg == "--gate-pre" && a + 1 < argc) {
            // Gate on PRE-dedupe recall: "does the recognizer itself see the
            // feature".  Post-dedupe recall is additionally suppressed by the
            // generic-beats-specific dedupe defect (measured 2026-06-11:
            // drill_hole at 0.95 claims the shared cylinder and dedupe drops
            // counterbore/bore/pocket candidates) — gate that separately via
            // --gate once the dedupe specificity fix lands (plan B3/B1.4).
            gatePre    = std::atof(argv[++a]);
            hasGatePre = true;
        } else if (arg == "--exempt" && a + 1 < argc) {
            // Comma-separated skill ids excluded from gating (NOT from the
            // report).  For by-design low-confidence recognizers (ream 0.35,
            // spot_face 0.65 — both below the 0.7 analysis threshold).
            std::string list = argv[++a];
            std::size_t pos  = 0;
            while (pos != std::string::npos) {
                const std::size_t comma = list.find(',', pos);
                const std::string id    = list.substr(
                    pos, comma == std::string::npos ? std::string::npos
                                                    : comma - pos);
                if (!id.empty()) exempt.insert(id);
                pos = (comma == std::string::npos) ? std::string::npos
                                                   : comma + 1;
            }
        } else if (arg == "--scale" && a + 1 < argc) {
            scaleF = std::atof(argv[++a]);
        } else if (!arg.empty() && arg[0] != '-') {
            samplesPerSkill = std::max(1, std::atoi(arg.c_str()));
        } else {
            std::cerr << "Usage: koo_corpus [samples_per_skill=3] "
                         "[--gate <recall>] [--gate-pre <recall>] "
                         "[--exempt a,b] [--scale <f>]\n";
            return 2;
        }
    }
    if (scaleF <= 0.0) {
        std::cerr << "koo_corpus: --scale must be > 0\n";
        return 2;
    }

    namespace fs = std::filesystem;
    fs::path tmpDir;
    try {
        tmpDir = fs::temp_directory_path();
    } catch (const std::exception& e) {
        std::cerr << "koo_corpus: no temp directory: " << e.what() << "\n";
        return 2;
    }
    // Session tag keeps parallel invocations from clobbering each other's
    // temp STEP files (sample determinism is unaffected — seeds are fixed).
    const long long tag = static_cast<long long>(
        std::chrono::steady_clock::now().time_since_epoch().count());

    const auto& cases = corpusCases();
    std::vector<Agg> agg(cases.size());

    for (std::size_t k = 0; k < cases.size(); ++k) {
        const std::string id = cases[k].id;
        for (int i = 0; i < samplesPerSkill; ++i) {
            // Spec: deterministic seed = skill_index*1000 + sample_index.
            std::mt19937 rng(static_cast<std::mt19937::result_type>(
                k * 1000u + static_cast<unsigned>(i)));

            Agg& a = agg[k];
            ++a.n;

            json row;
            row["skill"]  = id;
            row["sample"] = i;

            // 1) Synthesize the self-labelled part.
            Sample s;
            std::string err;
            try {
                s = cases[k].gen(rng, scaleF);
            } catch (const Standard_Failure&) {
                err = "OCCT Standard_Failure during apply";
            } catch (const std::exception& e) {
                err = std::string("apply failed: ") + e.what();
            } catch (...) {
                err = "apply failed: unknown exception";
            }
            row["params"] = s.params.is_null() ? json::object() : s.params;

            // 2) STEP round-trip (strips metadata → geometric recovery only).
            bool leak = false;
            std::vector<skill::RecognizedFeature> rawCands, deduped;
            if (err.empty()) {
                const fs::path p = tmpDir /
                    ("koo_corpus_" + std::to_string(tag) + "_" +
                     std::to_string(k) + "_" + std::to_string(i) + ".step");
                std::string ioErr;
                if (!io::StepIO::write(s.shape, p, ioErr)) {
                    err = "STEP write failed: " + ioErr;
                } else {
                    auto reim = io::StepIO::read(p, ioErr);
                    std::error_code ec;
                    fs::remove(p, ec);
                    if (!reim) {
                        err = "STEP read failed: " + ioErr;
                    } else {
                        skill::Workpiece wp(*reim);
                        leak = !wp.features().empty();   // must be 0 (non-circular)
                        // 3) The official RE pipeline.
                        try {
                            rawCands = re::analyze(wp);
                            deduped  = re::dedupe(rawCands);
                        } catch (const Standard_Failure&) {
                            err = "OCCT Standard_Failure during analyze";
                        } catch (const std::exception& e) {
                            err = std::string("analyze failed: ") + e.what();
                        }
                    }
                }
            }

            // 4) Score.
            Eval e, ePre;
            if (err.empty()) {
                e    = evaluate(s, id, aliasesFor(id), deduped);
                ePre = evaluate(s, id, kNoAliases, rawCands);   // pre-dedupe: strict id
            }
            const bool recognized = err.empty() && !leak && e.matched;

            row["recognized"]   = recognized;
            row["matched_conf"] = e.found ? json(e.conf) : json();
            row["dim_err_mm"]   = (e.found && std::isfinite(e.dimErr))
                                      ? json(e.dimErr) : json();
            row["pos_err_mm"]   = (e.found && e.posErr >= 0.0 &&
                                   std::isfinite(e.posErr))
                                      ? json(e.posErr) : json();
            row["metadata_leak"]      = leak;
            row["matched_pre_dedupe"] = err.empty() && !leak && ePre.matched;
            if (!deduped.empty()) {
                row["top_skill"] = deduped.front().skill_id;
                row["top_conf"]  = deduped.front().confidence;
            } else {
                row["top_skill"] = json();
                row["top_conf"]  = json();
            }
            if (!err.empty()) row["error"] = err;
            std::cout << row.dump() << "\n";

            if (!err.empty())              ++a.errors;
            if (leak)                      ++a.leaks;
            if (recognized)                ++a.hits;
            if (err.empty() && !leak && ePre.matched) ++a.hitsPre;
            if (e.found && std::isfinite(e.dimErr)) {
                a.errSum += e.dimErr;
                a.errMax  = std::max(a.errMax, e.dimErr);
                ++a.errN;
            }
        }
    }

    // ── Per-skill summary (stderr keeps stdout pure JSONL) ────────────────
    std::ostringstream sum;
    sum << "=== koo_corpus summary: samples_per_skill=" << samplesPerSkill
        << ", scale=" << scaleF << ", conf>=" << kConfThreshold << " ===\n";
    sum << std::left << std::setw(24) << "skill"
        << std::right << std::setw(4)  << "n"
        << std::setw(9)  << "recall"
        << std::setw(12) << "recall_pre"
        << std::setw(14) << "mean_dim_err"
        << std::setw(13) << "max_dim_err"
        << std::setw(7)  << "leaks"
        << std::setw(7)  << "errs" << "\n";
    sum << std::fixed;
    std::vector<std::string> gateFails;
    for (std::size_t k = 0; k < cases.size(); ++k) {
        const Agg& a = agg[k];
        const double recall    = (a.n > 0) ? double(a.hits)    / a.n : 0.0;
        const double recallPre = (a.n > 0) ? double(a.hitsPre) / a.n : 0.0;
        sum << std::left << std::setw(24) << cases[k].id
            << std::right << std::setw(4) << a.n
            << std::setw(9)  << std::setprecision(3) << recall
            << std::setw(12) << std::setprecision(3) << recallPre;
        if (a.errN > 0) {
            sum << std::setw(14) << std::setprecision(4) << (a.errSum / a.errN)
                << std::setw(13) << std::setprecision(4) << a.errMax;
        } else {
            sum << std::setw(14) << "-" << std::setw(13) << "-";
        }
        sum << std::setw(7) << a.leaks << std::setw(7) << a.errors << "\n";
        const bool isExempt = exempt.count(cases[k].id) > 0;
        if (!isExempt && hasGate && recall < gate)
            gateFails.push_back(std::string(cases[k].id) + " (recall)");
        if (!isExempt && hasGatePre && recallPre < gatePre)
            gateFails.push_back(std::string(cases[k].id) + " (recall_pre)");
    }
    std::cerr << sum.str();
    if (!exempt.empty()) {
        std::cerr << "exempt from gating:";
        for (const auto& id : exempt) std::cerr << " " << id;
        std::cerr << "\n";
    }

    if (hasGate || hasGatePre) {
        if (!gateFails.empty()) {
            std::cerr << "\nGATE FAIL: ";
            for (std::size_t j = 0; j < gateFails.size(); ++j) {
                if (j > 0) std::cerr << ", ";
                std::cerr << gateFails[j];
            }
            std::cerr << "\n";
            return 1;
        }
        std::cerr << "\nGATE PASS";
        if (hasGate)    std::cerr << " (recall >= "     << gate    << ")";
        if (hasGatePre) std::cerr << " (recall_pre >= " << gatePre << ")";
        std::cerr << "\n";
    }
    return 0;
}
