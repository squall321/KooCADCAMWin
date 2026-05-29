// @lat: [[process/test-strategy#skill round-trip]]
//
// gusset_install skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of undersized plate / insufficient weld, metadata-only
// recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gusset_install.hpp"

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

skill::gusset_install::Input goodInput()
{
    skill::gusset_install::Input in;
    in.gusset_size_mm = 300.0;
    in.weld_length_mm = 800.0;     // > 2 × 300 = 600
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillGussetInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::gusset_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("gusset_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (gusset 50 mm < 100 mm minimum) ────────
TEST(SkillGussetInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.gusset_size_mm = 50.0;
    in.weld_length_mm = 200.0;
    auto r = skill::gusset_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-GUS-SIZE"));
    EXPECT_THROW(skill::gusset_install::apply(*stock, in), skill::SkillError);

    // Weld too short — 2 × size = 600, set weld = 400.
    auto in2 = goodInput();
    in2.gusset_size_mm = 300.0;
    in2.weld_length_mm = 400.0;
    auto r2 = skill::gusset_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-GUS-WELD"));
}

// ─── 3. Signature records kind + size/weld_length ────────────────────────
TEST(SkillGussetInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.gusset_size_mm = 400.0;
    in.weld_length_mm = 1000.0;

    auto out = skill::gusset_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("gusset_install"));
    EXPECT_NEAR(out.signature.pattern["gusset_size_mm"].get<double>(),
                400.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["weld_length_mm"].get<double>(),
                1000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["weld_ratio"].get<double>(),
                1000.0 / 400.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillGussetInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::gusset_install::apply(*stock, goodInput());

    auto cands = skill::gusset_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["gusset_size_mm"].get<double>(),
                300.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::gusset_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "gusset_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: longer weld ⇒ longer SMAW cycle time ────────────────────
TEST(SkillGussetInstall, LongerWeldIncreasesCycleTime)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto shortIn = goodInput();
    shortIn.gusset_size_mm = 300.0;
    shortIn.weld_length_mm = 700.0;
    auto shortOut = skill::gusset_install::apply(*stock, shortIn);

    auto longIn = goodInput();
    longIn.gusset_size_mm = 300.0;
    longIn.weld_length_mm = 1100.0;       // borderline of 4×; still passes (info-only)
    auto longOut = skill::gusset_install::apply(*stock, longIn);

    EXPECT_GT(longOut.signature.tooling.est_cycle_time_s,
              shortOut.signature.tooling.est_cycle_time_s);
    EXPECT_EQ(shortOut.signature.tooling.tool_type,
              std::string("stick_welder"));
}
