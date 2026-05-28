// @lat: [[process/test-strategy#skill round-trip]]
//
// knurl — metadata-only surface texture (geometry unchanged).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/knurl.hpp"

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

skill::knurl::Input goodInput()
{
    skill::knurl::Input in;
    in.start_z_mm = 4.0;
    in.end_z_mm   = 16.0;
    in.pitch_mm   = 1.0;
    in.depth_mm   = 0.3;
    in.pattern    = "diamond";
    return in;
}

}  // namespace

// ─── 1. apply does NOT change geometry (metadata-only) ────────────────────
TEST(SkillKnurl, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();

    auto out = skill::knurl::apply(*stock, goodInput());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 2. DFM rejects depth ≥ pitch/2 (ridges intersect) ────────────────────
TEST(SkillKnurl, ValidateRejectsDepthExceedingPitchHalf)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    skill::knurl::Input in = goodInput();
    in.pitch_mm = 1.0;
    in.depth_mm = 0.7;          // > pitch/2 = 0.5
    auto r = skill::knurl::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-KNURL") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::knurl::apply(*stock, in), skill::SkillError);
}

// ─── 3. signature records kind + pattern ──────────────────────────────────
TEST(SkillKnurl, SignatureRecordsKind)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto out = skill::knurl::apply(*stock, goodInput());
    EXPECT_EQ(out.signature.skill_id, std::string("knurl"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("knurl"));
    EXPECT_EQ(out.signature.pattern["pattern"].get<std::string>(),
              std::string("diamond"));
}

// ─── 4. recognize via metadata replay ─────────────────────────────────────
TEST(SkillKnurl, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto out = skill::knurl::apply(*stock, goodInput());

    auto cands = skill::knurl::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["pattern"].get<std::string>(),
              std::string("diamond"));

    // Stripped workpiece → no recognition possible (geometry unchanged).
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::knurl::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. specific — pitch/depth round-trip exactly ─────────────────────────
TEST(SkillKnurl, PitchAndDepthRoundTrip)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    skill::knurl::Input in = goodInput();
    in.pitch_mm = 1.2;
    in.depth_mm = 0.45;
    in.pattern  = "helical_right";
    auto out = skill::knurl::apply(*stock, in);

    auto cands = skill::knurl::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].recovered_params["pitch_mm"].get<double>(),
                in.pitch_mm, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(),
                in.depth_mm, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["pattern"].get<std::string>(),
              std::string("helical_right"));
}
