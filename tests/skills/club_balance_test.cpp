// @lat: [[process/test-strategy#skill round-trip]]
//
// club_balance skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of malformed swing-weight code, metadata-only recognition,
// and a skill-specific assertion that the swing-weight scale is monotonic.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/club_balance.hpp"

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

skill::club_balance::Input goodInput()
{
    skill::club_balance::Input in;
    in.swing_weight = "D2";
    in.head_mass_g  = 200.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillClubBalance, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::club_balance::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("club_balance"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillClubBalance, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // Malformed swing-weight code.
    auto in = goodInput();
    in.swing_weight = "Z9";       // 'Z' not in A-F
    auto r = skill::club_balance::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-CLUB-SW"));
    EXPECT_THROW(skill::club_balance::apply(*stock, in), skill::SkillError);

    // Head mass out of physical range.
    auto in2 = goodInput();
    in2.head_mass_g = 50.0;       // < 150 g floor
    auto r2 = skill::club_balance::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CLUB-MASS"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillClubBalance, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.swing_weight = "D5";
    in.head_mass_g  = 205.0;

    auto out = skill::club_balance::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("club_balance"));
    EXPECT_EQ(out.signature.pattern["swing_weight"].get<std::string>(),
              std::string("D5"));
    EXPECT_NEAR(out.signature.pattern["head_mass_g"].get<double>(),
                205.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillClubBalance, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::club_balance::apply(*stock, goodInput());

    auto cands = skill::club_balance::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["swing_weight"].get<std::string>(),
              std::string("D2"));
    EXPECT_NEAR(cands[0].recovered_params["head_mass_g"].get<double>(),
                200.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::club_balance::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "club_balance cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: swing-weight scale is monotonic (D2 < D5 < E5) ─────────
TEST(SkillClubBalance, SwingWeightScaleMonotonic)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto inA = goodInput(); inA.swing_weight = "D2";
    auto inB = goodInput(); inB.swing_weight = "D5";
    auto inC = goodInput(); inC.swing_weight = "E5";

    auto outA = skill::club_balance::apply(*stock, inA);
    auto outB = skill::club_balance::apply(*stock, inB);
    auto outC = skill::club_balance::apply(*stock, inC);

    const int scaleA = outA.signature.pattern["swing_weight_scale"].get<int>();
    const int scaleB = outB.signature.pattern["swing_weight_scale"].get<int>();
    const int scaleC = outC.signature.pattern["swing_weight_scale"].get<int>();

    EXPECT_LT(scaleA, scaleB);
    EXPECT_LT(scaleB, scaleC);
}
