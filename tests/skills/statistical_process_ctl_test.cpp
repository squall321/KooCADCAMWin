// @lat: [[process/test-strategy#skill round-trip]]
//
// statistical_process_ctl — SPC Cp/Cpk computation skill.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (sigma <= 0, USL <= LSL, n < 1)
//   3. signature records kind + mean + sigma + USL + LSL + cp + cpk
//   4. recognize metadata replay
//   5. specific: Cp and Cpk formulas + off-centre case

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/statistical_process_ctl.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillStatisticalProcessCtl, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::statistical_process_ctl::Input in;
    in.characteristic = "bore_dia";
    in.mean           = 10.00;
    in.sigma          = 0.10;
    in.USL            = 10.50;
    in.LSL            = 9.50;
    in.sample_size    = 30;

    auto out = skill::statistical_process_ctl::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillStatisticalProcessCtl, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // sigma <= 0 → error.
    skill::statistical_process_ctl::Input in;
    in.mean        = 10.0;
    in.sigma       = 0.0;        // INVALID
    in.USL         = 11.0;
    in.LSL         = 9.0;
    in.sample_size = 30;
    auto r = skill::statistical_process_ctl::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::statistical_process_ctl::apply(*stock, in),
                 skill::SkillError);

    // USL <= LSL → error.
    skill::statistical_process_ctl::Input in2;
    in2.mean        = 10.0;
    in2.sigma       = 0.1;
    in2.USL         = 9.0;
    in2.LSL         = 11.0;
    in2.sample_size = 30;
    auto r2 = skill::statistical_process_ctl::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);

    // sample_size < 1 → error.
    skill::statistical_process_ctl::Input in3;
    in3.mean        = 10.0;
    in3.sigma       = 0.1;
    in3.USL         = 11.0;
    in3.LSL         = 9.0;
    in3.sample_size = 0;
    auto r3 = skill::statistical_process_ctl::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records kind + key params + computed cp/cpk ────────────
TEST(SkillStatisticalProcessCtl, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::statistical_process_ctl::Input in;
    in.characteristic = "shaft_dia";
    in.mean           = 10.00;     // centred
    in.sigma          = 0.10;
    in.USL            = 10.60;     // total range 1.2, ±0.6
    in.LSL            =  9.40;
    in.sample_size    = 50;

    auto out = skill::statistical_process_ctl::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("statistical_process_ctl"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("statistical_process_ctl"));
    EXPECT_NEAR(out.signature.pattern["mean"].get<double>(),  10.00, 1e-9);
    EXPECT_NEAR(out.signature.pattern["sigma"].get<double>(),  0.10, 1e-9);
    EXPECT_NEAR(out.signature.pattern["USL"].get<double>(),   10.60, 1e-9);
    EXPECT_NEAR(out.signature.pattern["LSL"].get<double>(),    9.40, 1e-9);

    // Cp = (10.60 − 9.40) / (6 · 0.10) = 1.2 / 0.6 = 2.0
    // Cpk (centred at 10.00) = min((10.60−10.00)/0.30, (10.00−9.40)/0.30)
    //                        = min(2.0, 2.0) = 2.0
    EXPECT_NEAR(out.signature.pattern["cp"].get<double>(),  2.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["cpk"].get<double>(), 2.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["verdict"].get<std::string>(),
              std::string("highly_capable"));
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillStatisticalProcessCtl, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::statistical_process_ctl::Input in;
    in.characteristic = "length";
    in.mean           = 50.0;
    in.sigma          = 0.5;
    in.USL            = 52.0;
    in.LSL            = 48.0;
    in.sample_size    = 30;
    auto out = skill::statistical_process_ctl::apply(*stock, in);

    auto cands = skill::statistical_process_ctl::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("statistical_process_ctl"));
    EXPECT_NEAR(cands[0].recovered_params["mean"].get<double>(),  50.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["sigma"].get<double>(),  0.5, 1e-9);

    // Workpiece without metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::statistical_process_ctl::recognize(raw).empty());
}

// ─── 5. SPC computes Cp/Cpk correctly (centred + off-centre + helper) ────
TEST(SkillStatisticalProcessCtl, ComputesCpCpkCorrectly)
{
    // ── Pure helper: centred case ────────────────────────────────────────
    {
        auto [cp, cpk] = skill::statistical_process_ctl::computeCpCpk(
            /*mean*/ 10.0, /*sigma*/ 0.1, /*USL*/ 10.6, /*LSL*/ 9.4);
        // Cp = 1.2 / 0.6 = 2.0; Cpk (centred) = 2.0.
        EXPECT_NEAR(cp,  2.0, 1e-9);
        EXPECT_NEAR(cpk, 2.0, 1e-9);
    }

    // ── Pure helper: off-centre case ────────────────────────────────────
    {
        // mean shifted 0.3 toward USL.  USL − μ = 0.3, μ − LSL = 0.9.
        // Cp  = (USL − LSL) / (6σ) = 1.2 / 0.6 = 2.0   (unchanged)
        // Cpk = min((USL − μ)/(3σ), (μ − LSL)/(3σ))
        //     = min(0.3/0.30, 0.9/0.30) = min(1.0, 3.0) = 1.0
        auto [cp, cpk] = skill::statistical_process_ctl::computeCpCpk(
            /*mean*/ 10.3, /*sigma*/ 0.1, /*USL*/ 10.6, /*LSL*/ 9.4);
        EXPECT_NEAR(cp,  2.0, 1e-9);
        EXPECT_NEAR(cpk, 1.0, 1e-9);
        EXPECT_LT(cpk, cp);   // off-centre always gives Cpk < Cp.
    }

    // ── End-to-end via apply(): incapable case + DFM info finding ───────
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::statistical_process_ctl::Input in;
    in.characteristic = "feature";
    in.mean           = 10.0;
    in.sigma          = 0.5;        // huge → Cp = 1.2/3.0 = 0.4 (incapable)
    in.USL            = 10.6;
    in.LSL            =  9.4;
    in.sample_size    = 30;
    auto r = skill::statistical_process_ctl::validate(*stock, in);
    EXPECT_TRUE(r.passed);          // info-level, not blocking
    EXPECT_TRUE(hasFinding(r, "DFM-SPC-INCAPABLE"));
    auto out = skill::statistical_process_ctl::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["cp"].get<double>(),  0.4, 1e-9);
    EXPECT_NEAR(out.signature.pattern["cpk"].get<double>(), 0.4, 1e-9);
    EXPECT_EQ(out.signature.pattern["verdict"].get<std::string>(),
              std::string("incapable"));
}
