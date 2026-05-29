// @lat: [[process/test-strategy#skill round-trip]]
//
// belt_replace skill — 5-case maintenance sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/belt_replace.hpp"

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

skill::belt_replace::Input goodInput()
{
    skill::belt_replace::Input in;
    in.belt_type = "V";
    in.belt_id   = "B-44";
    in.tension_n = 400.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillBeltReplace, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::belt_replace::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("belt_replace"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillBeltReplace, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.belt_id = "";                       // empty
    auto r = skill::belt_replace::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::belt_replace::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.tension_n = -10.0;                 // negative tension
    auto r2 = skill::belt_replace::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillBeltReplace, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.belt_type = "timing";
    in.belt_id   = "HTD-8M-1280";
    in.tension_n = 1200.0;

    auto out = skill::belt_replace::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("belt_replace"));
    EXPECT_EQ(out.signature.pattern["belt_type"].get<std::string>(),
              std::string("timing"));
    EXPECT_EQ(out.signature.pattern["belt_id"].get<std::string>(),
              std::string("HTD-8M-1280"));
    EXPECT_NEAR(out.signature.pattern["tension_n"].get<double>(), 1200.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillBeltReplace, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::belt_replace::apply(*stock, goodInput());

    auto cands = skill::belt_replace::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["belt_id"].get<std::string>(),
              std::string("B-44"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::belt_replace::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "belt_replace cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: timing belt selects timing-belt tension gauge ──────────
TEST(SkillBeltReplace, TimingBeltSelectsTimingGauge)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto vIn = goodInput();
    vIn.belt_type = "V";
    auto vOut = skill::belt_replace::apply(*stock, vIn);
    EXPECT_EQ(vOut.signature.tooling.tool_type,
              std::string("belt_tension_gauge"));

    auto tIn = goodInput();
    tIn.belt_type = "timing";
    tIn.belt_id   = "HTD-5M-500";
    auto tOut = skill::belt_replace::apply(*stock, tIn);
    EXPECT_EQ(tOut.signature.tooling.tool_type,
              std::string("timing_belt_tension_gauge"));
}
