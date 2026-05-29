// @lat: [[process/test-strategy#skill round-trip]]
//
// mems_dicing skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range kerf width, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mems_dicing.hpp"

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

skill::mems_dicing::Input goodInput()
{
    skill::mems_dicing::Input in;
    in.kerf_um     = 50.0;
    in.die_size_mm = 3.0;
    in.blade_type  = "diamond_resin";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillMemsDicing, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::mems_dicing::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("mems_dicing"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillMemsDicing, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.kerf_um = 5.0;                 // < 10 um
    auto r = skill::mems_dicing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-KERF"));
    EXPECT_THROW(skill::mems_dicing::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.die_size_mm = 50.0;           // > 30 mm
    auto r2 = skill::mems_dicing::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-DIE-SIZE"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillMemsDicing, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.kerf_um     = 80.0;
    in.die_size_mm = 5.5;
    in.blade_type  = "Ni_bonded";

    auto out = skill::mems_dicing::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("mems_dicing"));
    EXPECT_NEAR(out.signature.pattern["kerf_um"].get<double>(),     80.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["die_size_mm"].get<double>(),  5.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["blade_type"].get<std::string>(),
              std::string("Ni_bonded"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillMemsDicing, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::mems_dicing::apply(*stock, goodInput());

    auto cands = skill::mems_dicing::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["kerf_um"].get<double>(),
                50.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::mems_dicing::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "mems_dicing cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: stealth_laser ⇒ dry process + faster feed ───────────────
TEST(SkillMemsDicing, StealthLaserIsDryAndFaster)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto sawIn = goodInput();
    sawIn.blade_type = "diamond_resin";
    auto sawOut = skill::mems_dicing::apply(*stock, sawIn);
    EXPECT_EQ(sawOut.signature.tooling.tool_type,
              std::string("wafer_dicing_saw"));
    EXPECT_TRUE(sawOut.signature.pattern["rinse_required"].get<bool>());

    auto laserIn = goodInput();
    laserIn.blade_type = "stealth_laser";
    auto laserOut = skill::mems_dicing::apply(*stock, laserIn);
    EXPECT_EQ(laserOut.signature.tooling.tool_type,
              std::string("stealth_dicing_laser"));
    EXPECT_FALSE(laserOut.signature.pattern["rinse_required"].get<bool>());

    // Laser feed > blade feed → laser cycle time shorter.
    EXPECT_GT(laserOut.signature.pattern["feed_mm_per_s"].get<double>(),
              sawOut.signature.pattern["feed_mm_per_s"].get<double>());
    EXPECT_LT(laserOut.signature.tooling.est_cycle_time_s,
              sawOut.signature.tooling.est_cycle_time_s);
}
