// @lat: [[process/test-strategy#skill round-trip]]
//
// etch_silicon — DRIE / wet / dry silicon-etch metadata-only skill.
// Specific assertion: depth_um > 0 is ENFORCED as a hard error.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/etch_silicon.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

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

// ── 1. Apply: geometry unchanged (clone, volume preserved) ───────────────
TEST(SkillEtchSilicon, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::etch_silicon::Input in;
    in.etch_type   = "DRIE";
    in.depth_um    = 20.0;
    in.selectivity = 75.0;
    in.mask_id     = "M-DRIE-01";

    auto out = skill::etch_silicon::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. ValidateRejectsBadInput: zero depth, bad selectivity, empty mask ──
TEST(SkillEtchSilicon, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    // SPECIFIC: depth_um > 0 must be enforced as a hard error.
    skill::etch_silicon::Input zeroDepth;
    zeroDepth.etch_type   = "DRIE";
    zeroDepth.depth_um    = 0.0;
    zeroDepth.selectivity = 50.0;
    zeroDepth.mask_id     = "M-OK";
    auto r1 = skill::etch_silicon::validate(*stock, zeroDepth);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::etch_silicon::apply(*stock, zeroDepth),
                 skill::SkillError);

    skill::etch_silicon::Input negDepth = zeroDepth;
    negDepth.depth_um = -5.0;
    auto r1b = skill::etch_silicon::validate(*stock, negDepth);
    EXPECT_FALSE(r1b.passed);

    skill::etch_silicon::Input zeroSel;
    zeroSel.etch_type   = "DRIE";
    zeroSel.depth_um    = 10.0;
    zeroSel.selectivity = 0.0;
    zeroSel.mask_id     = "M-OK";
    auto r2 = skill::etch_silicon::validate(*stock, zeroSel);
    EXPECT_FALSE(r2.passed);

    skill::etch_silicon::Input emptyMask;
    emptyMask.etch_type   = "DRIE";
    emptyMask.depth_um    = 10.0;
    emptyMask.selectivity = 50.0;
    emptyMask.mask_id     = "";
    auto r3 = skill::etch_silicon::validate(*stock, emptyMask);
    EXPECT_FALSE(r3.passed);

    skill::etch_silicon::Input badType;
    badType.etch_type   = "ion_milling";       // not in {DRIE, wet, dry}
    badType.depth_um    = 10.0;
    badType.selectivity = 50.0;
    badType.mask_id     = "M-OK";
    auto r4 = skill::etch_silicon::validate(*stock, badType);
    EXPECT_FALSE(r4.passed);
}

// ── 3. SignatureRecordsKind + key params ─────────────────────────────────
TEST(SkillEtchSilicon, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(200.0, 0.775, "silicon_wafer");

    skill::etch_silicon::Input in;
    in.etch_type   = "wet";
    in.depth_um    = 15.0;
    in.selectivity = 100.0;
    in.mask_id     = "M-KOH-42";

    auto out = skill::etch_silicon::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("etch_silicon"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("etch_silicon"));
    EXPECT_EQ(out.signature.pattern["etch_type"].get<std::string>(),
              std::string("wet"));
    EXPECT_NEAR(out.signature.pattern["depth_um"].get<double>(),
                15.0, 1e-12);
    EXPECT_NEAR(out.signature.pattern["selectivity"].get<double>(),
                100.0, 1e-12);
    EXPECT_EQ(out.signature.pattern["mask_id"].get<std::string>(),
              std::string("M-KOH-42"));
    EXPECT_TRUE(out.signature.pattern["is_subtractive"].get<bool>());
    // mask_consumed_um = depth / selectivity = 15 / 100 = 0.15
    EXPECT_NEAR(out.signature.pattern["mask_consumed_um"].get<double>(),
                0.15, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("etch_chamber"));
}

// ── 4. RecognizeMetadataReplay ───────────────────────────────────────────
TEST(SkillEtchSilicon, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::etch_silicon::Input in;
    in.etch_type   = "DRIE";
    in.depth_um    = 30.0;
    in.selectivity = 75.0;
    in.mask_id     = "M-RECG-DRIE";
    auto out = skill::etch_silicon::apply(*stock, in);

    auto cands = skill::etch_silicon::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["mask_id"].get<std::string>(),
              std::string("M-RECG-DRIE"));
    EXPECT_NEAR(cands[0].recovered_params["depth_um"].get<double>(),
                30.0, 1e-12);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::etch_silicon::recognize(raw).empty());
}

// ── 5. SPECIFIC: deep etch with low selectivity emits info ───────────────
//     (selectivity < 10 AND depth_um > 50 → mask consumption warning)
TEST(SkillEtchSilicon, LowSelectivityDeepEtchEmitsInfo)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::etch_silicon::Input in;
    in.etch_type   = "dry";
    in.depth_um    = 100.0;            // > 50 μm
    in.selectivity = 5.0;              // < 10
    in.mask_id     = "M-DRY-01";

    auto r = skill::etch_silicon::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "low-selectivity warning is advisory, not blocking";
    EXPECT_TRUE(hasFinding(r, "DFM-ETCH-SELECTIVITY"));
    EXPECT_NO_THROW(skill::etch_silicon::apply(*stock, in));
}
