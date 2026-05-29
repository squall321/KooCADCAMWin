// @lat: [[process/test-strategy#skill round-trip]]
//
// curved_lip_around_face — REAL-behavior tests for the perimeter lip macro
// (rectangular-ring extrusion that derives its size from the entry-face bbox).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/curved_lip_around_face.hpp"

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

skill::curved_lip_around_face::Input goodInput()
{
    skill::curved_lip_around_face::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_class = "cover_lip";          // inset=2, T=1.5, H=2
    return in;
}

}  // namespace

// ─── Test 1: apply adds a ring volume matching the spec geometry ─────────
TEST(SkillCurvedLipAroundFace, ApplyAddsRingVolume)
{
    // 50×50 top face → outer L,W = 50-2*2 = 46.  inner = 46-2*1.5 = 43.
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    auto in = goodInput();
    const double V0 = volumeOf(stock->shape());

    auto out = skill::curved_lip_around_face::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double inset = skill::curved_lip_around_face::specInsetFor    ("cover_lip");
    const double T     = skill::curved_lip_around_face::specThicknessFor("cover_lip");
    const double H     = skill::curved_lip_around_face::specHeightFor   ("cover_lip");
    EXPECT_NEAR(inset, 2.0, 1e-9);
    EXPECT_NEAR(T,     1.5, 1e-9);
    EXPECT_NEAR(H,     2.0, 1e-9);

    const double outerL = 50.0 - 2.0 * inset;
    const double outerW = 50.0 - 2.0 * inset;
    const double innerL = outerL - 2.0 * T;
    const double innerW = outerW - 2.0 * T;

    // Ring area × height (added).
    const double expected = (outerL * outerW - innerL * innerW) * H;

    const double delta = volumeOf(out.workpiece->shape()) - V0;
    EXPECT_NEAR(delta, expected, expected * 0.05);
}

// ─── Test 2: validate rejects unknown spec + auto-detect inset overflow ──
TEST(SkillCurvedLipAroundFace, ValidateRejectsUnknownSpecAndContextOverflow)
{
    // (a) Unknown spec
    {
        auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
        auto in = goodInput();
        in.spec_class = "M99_lip";
        auto r = skill::curved_lip_around_face::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LIP-UNKNOWN") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::curved_lip_around_face::apply(*stock, in), skill::SkillError);
    }

    // (b) Auto-detect context overflow: 6×6 mm face cannot accommodate
    //     flange_lip (inset 5, T 2.5).  2·(5+2.5) = 15 ≥ 6.
    {
        auto stock = skill::createCuboidStock(6.0, 6.0, 10.0);
        auto in = goodInput();
        in.spec_class = "flange_lip";
        auto r = skill::curved_lip_around_face::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LIP-INSET") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::curved_lip_around_face::apply(*stock, in), skill::SkillError);
    }
}

// ─── Test 3: signature carries spec_class + derived face-derived params ──
TEST(SkillCurvedLipAroundFace, SignatureCarriesSpecKeyAndContextDerivedDims)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out = skill::curved_lip_around_face::apply(*stock, goodInput());

    const auto& sig = out.signature;
    EXPECT_EQ(sig.params.value("spec_class", std::string()), "cover_lip");
    EXPECT_EQ(sig.pattern.value("kind",        std::string()), "curved_lip_around_face");
    EXPECT_TRUE(sig.pattern.value("is_compound", false));
    EXPECT_EQ(sig.pattern.value("subfeature_count", 0), 6);
    EXPECT_NEAR(sig.pattern.value("inset_mm",     0.0), 2.0, 1e-9);
    EXPECT_NEAR(sig.pattern.value("thickness_mm", 0.0), 1.5, 1e-9);
    EXPECT_NEAR(sig.pattern.value("height_mm",    0.0), 2.0, 1e-9);
    // Outer length derived from face context (50 mm − 2 × inset).
    EXPECT_NEAR(sig.pattern.value("outer_l_mm",   0.0), 46.0, 1e-6);
    EXPECT_NEAR(sig.pattern.value("inner_l_mm",   0.0), 43.0, 1e-6);
}

// ─── Test 4: recognize returns the candidate with the right spec ─────────
TEST(SkillCurvedLipAroundFace, RecognizeReturnsCandidateWithSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out = skill::curved_lip_around_face::apply(*stock, goodInput());
    auto cands = skill::curved_lip_around_face::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("curved_lip_around_face"));
    EXPECT_GT(c.confidence, 0.5);
    EXPECT_EQ(c.recovered_params.value("spec_class", std::string()), "cover_lip");
}

// ─── Test 5: context-detection — apply DOES derive dims from the workpiece
//             face (i.e. a different stock size yields different outer dims).
TEST(SkillCurvedLipAroundFace, ContextDetectionAdaptsToFaceSize)
{
    // First: 50×50 stock → outer = 46×46.
    auto stockA = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto outA   = skill::curved_lip_around_face::apply(*stockA, goodInput());
    EXPECT_NEAR(outA.signature.pattern.value("outer_l_mm", 0.0), 46.0, 1e-6);
    EXPECT_NEAR(outA.signature.pattern.value("outer_w_mm", 0.0), 46.0, 1e-6);

    // Second: 80×60 stock → outer = 76×56.
    auto stockB = skill::createCuboidStock(80.0, 60.0, 10.0);
    auto outB   = skill::curved_lip_around_face::apply(*stockB, goodInput());
    EXPECT_NEAR(outB.signature.pattern.value("outer_l_mm", 0.0), 76.0, 1e-6);
    EXPECT_NEAR(outB.signature.pattern.value("outer_w_mm", 0.0), 56.0, 1e-6);
    // i.e. the skill autonomously read the face bbox from the workpiece
    // context — the caller never supplied length/width.
}
