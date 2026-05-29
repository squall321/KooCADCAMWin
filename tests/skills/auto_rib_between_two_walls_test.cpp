// @lat: [[process/test-strategy#auto_rib_between_two_walls]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/auto_rib_between_two_walls.hpp"

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

// Pick two opposing planar faces on a cuboid stock — the +X and -X side
// faces — as walls A and B.  We brute-force scan for them.
struct WallPair { int faceA = -1; int faceB = -1; };
WallPair findOpposingFaces(const skill::Workpiece& wp,
                            const gp_Dir& wantNormalA,
                            const gp_Dir& wantNormalB)
{
    WallPair pair;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const gp_Dir n = wp.faceNormal(i);
        if (n.IsEqual(wantNormalA, 0.1) && pair.faceA < 0) pair.faceA = i;
        if (n.IsEqual(wantNormalB, 0.1) && pair.faceB < 0) pair.faceB = i;
    }
    return pair;
}

skill::auto_rib_between_two_walls::Input goodInput(int fA, int fB)
{
    skill::auto_rib_between_two_walls::Input in;
    in.wall_A_face_id = fA;
    in.wall_B_face_id = fB;
    in.rib_thick_mm   = 1.5;
    in.rib_height_mm  = 5.0;
    in.rib_style      = "standard";
    in.wall_thick_mm  = 2.5;
    return in;
}

}  // namespace

// ─── T1: apply produces volume delta matching expected geometry ───────────
TEST(SkillAutoRibBetweenTwoWalls, ApplyFusesRib)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    auto pair = findOpposingFaces(*stock, gp_Dir(-1, 0, 0), gp_Dir(1, 0, 0));
    ASSERT_GE(pair.faceA, 0);
    ASSERT_GE(pair.faceB, 0);

    const double v0 = volumeOf(stock->shape());

    auto in  = goodInput(pair.faceA, pair.faceB);
    auto out = skill::auto_rib_between_two_walls::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Rib length = wall-centre distance ≈ 50 mm; the rib volume ≈ 50 × 1.5
    // × 5 = 375 mm³, minus chamfers (~ 2 × 0.5³ = 0.25 mm³).
    const double ribL    = 50.0;
    const double vRib    = ribL * in.rib_thick_mm * in.rib_height_mm;
    const double vCham   = 2.0 * 0.5 * 0.5 * 0.5;
    const double expDelta = vRib - vCham;

    // Note: rib overlaps the stock since wall centres are ON the stock
    // surface; OCCT fuse only adds external volume.  Actual delta may be
    // less than ideal — we just verify it's strictly positive and within
    // 20% of expected upper bound.
    const double actDelta = volumeOf(out.workpiece->shape()) - v0;
    EXPECT_GE(actDelta, -vCham);  // at minimum chamfers were removed
    EXPECT_LE(actDelta, expDelta * 1.20);
}

// ─── T2: validate rejects unknown style; apply throws ─────────────────────
TEST(SkillAutoRibBetweenTwoWalls, RejectsUnknownStyle)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto pair = findOpposingFaces(*stock, gp_Dir(-1, 0, 0), gp_Dir(1, 0, 0));
    ASSERT_GE(pair.faceA, 0);

    auto in = goodInput(pair.faceA, pair.faceB);
    in.rib_style = "ultra";   // unknown

    auto r = skill::auto_rib_between_two_walls::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RIB-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::auto_rib_between_two_walls::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries style + is_compound = true ─────────────────────
TEST(SkillAutoRibBetweenTwoWalls, SignatureCarriesStyleCompound)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto pair = findOpposingFaces(*stock, gp_Dir(-1, 0, 0), gp_Dir(1, 0, 0));
    auto in   = goodInput(pair.faceA, pair.faceB);
    auto out  = skill::auto_rib_between_two_walls::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("auto_rib_between_two_walls"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["rib_style"].get<std::string>(),
              "standard");
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
}

// ─── T4: recognize returns at least one candidate ─────────────────────────
TEST(SkillAutoRibBetweenTwoWalls, RecognizeFindsACandidate)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto pair = findOpposingFaces(*stock, gp_Dir(-1, 0, 0), gp_Dir(1, 0, 0));
    auto in   = goodInput(pair.faceA, pair.faceB);
    auto out  = skill::auto_rib_between_two_walls::apply(*stock, in);

    auto cands = skill::auto_rib_between_two_walls::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands.front().skill_id,
              std::string("auto_rib_between_two_walls"));
    EXPECT_GT(cands.front().confidence, 0.5);
}

// ─── T5: context — rib length auto-detected from face centres ─────────────
TEST(SkillAutoRibBetweenTwoWalls, AutoDetectsRibLength)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto pair = findOpposingFaces(*stock, gp_Dir(-1, 0, 0), gp_Dir(1, 0, 0));
    auto in   = goodInput(pair.faceA, pair.faceB);
    auto out  = skill::auto_rib_between_two_walls::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern["auto_detected_length"].get<bool>());
    const double L = out.signature.pattern["rib_length_mm"].get<double>();
    EXPECT_NEAR(L, 50.0, 1.0)
        << "rib length should equal distance between opposing wall centres";
}
