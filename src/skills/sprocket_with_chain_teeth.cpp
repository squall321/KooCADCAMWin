// @lat: [[engine/skills#sprocket_with_chain_teeth]]

#include "sprocket_with_chain_teeth.hpp"

#include "Workpiece.hpp"
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
#include <array>
#include <cmath>

namespace koocadcam::skill::sprocket_with_chain_teeth {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct ChainEntry { const char* size; double pitch_mm; double roller_dia_mm; };

constexpr std::array<ChainEntry, 6> kChainTable {{
    { "25",  6.350,  3.300 },
    { "35",  9.525,  5.080 },
    { "40", 12.700,  7.920 },
    { "50", 15.875, 10.160 },
    { "60", 19.050, 11.910 },
    { "80", 25.400, 15.880 },
}};

}  // namespace

bool lookupChain(const std::string& ansi, ChainSize& out)
{
    for (const auto& e : kChainTable) {
        if (ansi == e.size) {
            out.pitch_mm = e.pitch_mm;
            out.roller_dia_mm = e.roller_dia_mm;
            return true;
        }
    }
    return false;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.blank_dia_mm <= 0.0 || in.blank_thick_mm <= 0.0 ||
        in.bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sprocket_with_chain_teeth: positive blank/bore required");
        return r;
    }

    ChainSize cs;
    if (!lookupChain(in.ansi_chain_size, cs)) {
        r.add("DFM-CHAIN-SIZE", "error",
              "sprocket_with_chain_teeth: unknown chain '" +
              in.ansi_chain_size + "' (supported: 25/35/40/50/60/80)");
        return r;
    }

    if (in.num_teeth < 9 || in.num_teeth > 120) {
        r.add("DFM-CHAIN-N", "error",
              "sprocket_with_chain_teeth: num_teeth " +
              std::to_string(in.num_teeth) + " outside [9, 120]");
    }

    if (in.num_teeth >= 9) {
        const double pd = cs.pitch_mm /
            std::sin(M_PI / static_cast<double>(in.num_teeth));
        const double od = pd + cs.roller_dia_mm;
        if (od > in.blank_dia_mm + 1e-3) {
            r.add("DFM-CHAIN-OD", "error",
                  "sprocket_with_chain_teeth: derived OD " +
                  std::to_string(od) +
                  " mm exceeds blank_dia " + std::to_string(in.blank_dia_mm));
        }
        if (in.bore_dia_mm >= pd - cs.roller_dia_mm) {
            r.add("DFM-CHAIN-BORE", "error",
                  "sprocket_with_chain_teeth: bore_dia " +
                  std::to_string(in.bore_dia_mm) +
                  " too large vs pitch circle");
        }
    }

    if (in.bore_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "sprocket_with_chain_teeth: bore_dia < 0.8 mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sprocket_with_chain_teeth DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    ChainSize cs;
    lookupChain(in.ansi_chain_size, cs);

    const int N = in.num_teeth;
    const double pitchDia = cs.pitch_mm / std::sin(M_PI / static_cast<double>(N));
    const double pitchR   = pitchDia / 2.0;
    const double rollerR  = cs.roller_dia_mm / 2.0;
    const double boreR    = in.bore_dia_mm / 2.0;
    const double Z        = in.blank_thick_mm;
    const double overhang = 0.05;

    // Tool 1: through bore.
    const gp_Ax2 axBore(gp_Pnt(0, 0, -overhang), gp::DZ());
    const TopoDS_Shape boreTool = pr::cylinder(axBore, boreR, Z + 2.0 * overhang);

    // Tools 2..N+1: N pocket cylindrical cutters, each centered on a tooth
    // pitch point.  Each pocket cuts a half-roller seat through full thickness.
    std::vector<TopoDS_Shape> tools;
    tools.reserve(static_cast<size_t>(N) + 1);
    tools.push_back(boreTool);
    for (int i = 0; i < N; ++i) {
        const double th = (2.0 * M_PI * static_cast<double>(i)) /
                          static_cast<double>(N);
        const gp_Pnt c(pitchR * std::cos(th), pitchR * std::sin(th), -overhang);
        const gp_Ax2 ax(c, gp::DZ());
        // Roller-seat pocket: cylindrical hole of roller_dia (the chain
        // tooth pitch valley); cuts through the rim creating tooth gaps.
        tools.push_back(pr::cylinder(ax, rollerR, Z + 2.0 * overhang));
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    const double od = pitchDia + cs.roller_dia_mm;
    const int subCount = 1 + N;

    json params = {
        { "blank_dia_mm",     in.blank_dia_mm },
        { "blank_thick_mm",   in.blank_thick_mm },
        { "ansi_chain_size",  in.ansi_chain_size },
        { "num_teeth",        in.num_teeth },
        { "bore_dia_mm",      in.bore_dia_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     subCount },
        { "blank_dia_mm",         in.blank_dia_mm },
        { "blank_thick_mm",       in.blank_thick_mm },
        { "ansi_chain_size",      in.ansi_chain_size },
        { "chain_pitch_mm",       cs.pitch_mm },
        { "roller_dia_mm",        cs.roller_dia_mm },
        { "num_teeth",            N },
        { "pitch_dia_mm",         pitchDia },
        { "outer_dia_mm",         od },
        { "bore_dia_mm",          in.bore_dia_mm },
        { "standard",             "ANSI B29.1" },
    };

    ToolingMeta tooling;
    tooling.tool_type    = "drill;sprocket_form_cutter";
    tooling.tool_dia_mm  = cs.roller_dia_mm;
    tooling.tool_length_mm = in.blank_thick_mm + 10.0;
    tooling.tool_material = "carbide";
    tooling.cutting_speed_sfm = 250.0;
    tooling.stock_removed_mm3 =
        M_PI * boreR * boreR * Z +
        N * M_PI * rollerR * rollerR * Z;
    tooling.est_cycle_time_s = std::max(30.0, N * 2.0 + 30.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },         { "tool_dia_mm", in.bore_dia_mm } },
            { { "tool_type", "form_drill" },    { "tool_dia_mm", cs.roller_dia_mm },
              { "pass_count", N }, { "note", "tooth pocket per ANSI B29.1" } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::sprocket_with_chain_teeth applied: ANSI-{} N={} PD={}",
                  in.ansi_chain_size, N, pitchDia);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Primary geometric: count cylindrical faces with axis ≈ Z.  A sprocket has
// 1 bore + N tooth-pocket cylinders, all aligned on the gear axis.  We
// identify the bore as the smallest-radius cyl and use the count of other
// Z-axis cylinders as N estimate.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    struct CylRecord { double r; double cx; double cy; };
    std::vector<CylRecord> zCyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder cy = s.Cylinder();
        const gp_Dir d = cy.Axis().Direction();
        if (std::abs(std::abs(d.Z()) - 1.0) > 1e-2) continue;
        const gp_Pnt loc = cy.Axis().Location();
        zCyls.push_back({ cy.Radius(), loc.X(), loc.Y() });
    }

    if (zCyls.size() >= 4) {
        // Find bore: smallest radius near origin.
        std::sort(zCyls.begin(), zCyls.end(),
                  [](const CylRecord& a, const CylRecord& b) { return a.r < b.r; });
        const double boreR = zCyls.front().r;

        // Tooth pockets share a common radius; group remaining cyls by
        // radius and pick the largest group.
        std::vector<int> hist;
        std::vector<double> bucketR;
        for (size_t k = 1; k < zCyls.size(); ++k) {
            bool placed = false;
            for (size_t b = 0; b < bucketR.size(); ++b) {
                if (std::abs(zCyls[k].r - bucketR[b]) < 0.3) {
                    hist[b]++;
                    placed = true;
                    break;
                }
            }
            if (!placed) { bucketR.push_back(zCyls[k].r); hist.push_back(1); }
        }
        int bestBucket = -1, bestCount = 0;
        for (size_t b = 0; b < hist.size(); ++b) {
            if (hist[b] > bestCount) { bestCount = hist[b]; bestBucket = static_cast<int>(b); }
        }

        if (bestBucket >= 0 && bestCount >= 9) {
            const double rollerR = bucketR[bestBucket];

            // Estimate pitch_dia from one pocket center distance to origin.
            double pitchR_sum = 0.0;
            int pitchR_n = 0;
            for (size_t k = 1; k < zCyls.size(); ++k) {
                if (std::abs(zCyls[k].r - rollerR) < 0.3) {
                    pitchR_sum += std::hypot(zCyls[k].cx, zCyls[k].cy);
                    ++pitchR_n;
                }
            }
            const double pitchR_est = (pitchR_n > 0) ? pitchR_sum / pitchR_n : 0.0;
            const int N = bestCount;

            // Derive chain size: P = 2 * pitchR * sin(pi/N)
            const double pitchEst = 2.0 * pitchR_est *
                                    std::sin(M_PI / static_cast<double>(N));
            // Match to chain table
            std::string chainSize = "";
            double bestErr = 1e9;
            for (const auto& e : kChainTable) {
                const double err = std::abs(e.pitch_mm - pitchEst);
                if (err < bestErr) { bestErr = err; chainSize = e.size; }
            }

            double xMin, yMin, zMin, xMax, yMax, zMax;
            wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

            json recovered = {
                { "blank_dia_mm",    std::max(xMax - xMin, yMax - yMin) },
                { "blank_thick_mm",  zMax - zMin },
                { "ansi_chain_size", chainSize },
                { "num_teeth",       N },
                { "bore_dia_mm",     2.0 * boreR },
            };
            json matched = {
                { "source",                "geometric_heuristic" },
                { "z_axis_cyl_count",      static_cast<int>(zCyls.size()) },
                { "pocket_count",          bestCount },
                { "estimated_pitch_mm",    pitchEst },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, 0.7, matched
            });
        }
    }

    // Fallback metadata
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", f.pattern.value("subfeature_count", 0) },
            { "num_teeth",        f.pattern.value("num_teeth", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::sprocket_with_chain_teeth
